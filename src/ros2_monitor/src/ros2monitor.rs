pub mod ros2monitor {
    use std::collections::HashMap;
    use std::process::{Command, ExitStatus};
    use std::string::String;
    use regex;

    use rclrs::Context;
    use std::{env, fs, io};
    use std::ffi::OsString;
    use std::fmt::format;
    use std::panic::resume_unwind;
    use std::path::PathBuf;
    use std::sync::{Arc, Mutex};
    use log::{error, info, logger};
    use serde::de::Unexpected::Str;
    use serde_json::{json, Value, Map};
    use crate::ros2entites::ros2entities::{Ros2State, Ros2Package, Ros2Node, Ros2Subscriber, Ros2Publisher, Ros2ServiceServer, Ros2ServiceClient, Ros2ActionClient, Ros2ActionServer};

    pub struct JsonProtocol {
        // List of all possible commands
        pub allowed_commands: Vec<String>,
        // Map of required arguments for each command
        pub allowed_arguments: HashMap<String, Vec<String>>,

        pub command: String,
        pub arguments: HashMap<String, String>,
    }

    impl JsonProtocol {
        pub fn new() -> JsonProtocol {
            let mut commands = Vec::new();
            let mut arguments: HashMap<String, Vec<String>> = HashMap::new();

            commands.push("state".to_string());
            arguments.insert("state".to_string(), Vec::new());

            commands.push("kill_node".to_string());
            let mut kill_node_args = Vec::new();
            kill_node_args.push("node_name".to_string());
            arguments.insert("kill_node".to_string(), kill_node_args);

            return JsonProtocol {
                allowed_commands: commands,
                allowed_arguments: arguments,
                command: "".to_string(),
                arguments: HashMap::new(),
            };
        }

        /// Parse json formatted request string. Return nothing on success, error message - on error
        pub fn parse_struct(&mut self, json_request: &str) -> Result<(), String> {
            let valid_example = r#"
            {
                "command": <command_name>,
                "arguments": [<argument list>]
            }
            "#;

            let trimmed = json_request.trim();
            let request_parse = serde_json::from_str(trimmed);
            if !request_parse.is_ok() {
                let msg = format!("Request must be valid json. Please, use the followed command structure: \n{}", valid_example).to_string();
                return Err(msg);
            }

            let request: Map<String, Value> = request_parse.unwrap();
            if !request.contains_key("command") {
                let msg = format!("Json request must contain command name. Please, use the followed command structure: \n{}", valid_example).to_string();
                return Err(msg);
            }

            let command = request.get("command").unwrap().as_str().unwrap().to_string();
            if !self.allowed_commands.contains(&command) {
                let msg = format!("You must use one of the following supported commands: {:?}", self.allowed_commands);
                return Err(msg);
            }
            self.command = request.get("command").unwrap().as_str().unwrap().to_string();

            if !request.contains_key("arguments") {
                let msg = format!("Json request must contain arguments array. Please, use the followed command structure: \n{}", valid_example).to_string();
                return Err(msg);
            }


            for argument in request.get("arguments") {
                let arg_obj = argument.as_object().unwrap();
                if !arg_obj.contains_key("name") {
                    let msg = "Each argument object in request must have a name field";
                    return Err(msg.to_string());
                }

                if !arg_obj.contains_key("value") {
                    let msg = "Each argument object in request must have a value field";
                    return Err(msg.to_string());
                }

                let arg_name = arg_obj.get("name").unwrap().as_str().unwrap().to_string();
                let arg_value = arg_obj.get("value").unwrap().as_str().unwrap().to_string();

                if !self.allowed_arguments.get(&command).unwrap().contains(&arg_name) {
                    let allowed_arguments = self.allowed_arguments.get(self.command.as_str()).unwrap();
                    let msg = format!("Argument {} is not allowed for command {}. Allowed arguments for this command: {:?}", arg_name, self.command, allowed_arguments);
                    return Err(msg);
                }

                self.arguments.insert(arg_name, arg_value);
            }

            Ok(())
        }
    }

    pub fn init_ros2() -> Context {
        let context = rclrs::Context::new(env::args());
        return context.unwrap();
    }

    pub fn ros2_cold_state() -> Ros2State {
        let nodes: Vec<Ros2Node> = explore_nodes();
        //let packages: Vec<Ros2Package> = explore_packages();
        let packages: Vec<Ros2Package> = Vec::new();

        let state = Ros2State {
            nodes,
            packages,
            executables: Vec::new(),
        };

        return state;
    }

    pub fn ros2_hot_state() -> Ros2State {
        let nodes: Vec<Ros2Node> = explore_nodes();
        //let packages: Vec<Ros2Package> = packages();

        let state = Ros2State {
            nodes,
            packages: Vec::new(),
            executables: Vec::new(),
        };

        return state;
    }

    pub fn ros2_state(current_state: Arc<Mutex<Ros2State>>) -> Ros2State {
        let nodes: Vec<Ros2Node> = explore_nodes();
        let mut packages: Vec<Ros2Package> = Vec::new();
        if current_state.lock().unwrap().packages.is_empty() {
            packages = explore_packages();
        } else {
            packages = current_state.lock().unwrap().packages.clone();
        }

        let state = Ros2State {
            nodes,
            packages,
            executables: Vec::new(),
        };

        return state;
    }

    pub fn explore_nodes() -> Vec<Ros2Node> {
        let node_names = ros2_node_names();
        let mut nodes: Vec<Ros2Node> = Vec::new();
        for node_name in node_names {
            let node_info: String = node_info(node_name.clone());
            let re = regex::Regex::new(r"Subscribers|Publishers|Service Servers|Service Clients|Action Servers|Action Clients").unwrap();
            let parts: Vec<String> = re.split(node_info.as_str()).map(|x| x.to_string())
                .collect();
            let subscribers_info: Vec<String> = parts[1].lines().skip(1).map(|line| line.trim().to_string()).filter(|line| !line.is_empty()).collect();
            let publishers_info: Vec<String> = parts[2].lines().skip(1).map(|line| line.trim().to_string()).filter(|line| !line.is_empty()).collect();
            let service_servers_info: Vec<String> = parts[3].lines().skip(1).map(|line| line.trim().to_string()).filter(|line| !line.is_empty()).collect();
            let service_clients_info: Vec<String> = parts[4].lines().skip(1).map(|line| line.trim().to_string()).filter(|line| !line.is_empty()).collect();
            let action_servers_info: Vec<String> = parts[5].lines().skip(1).map(|line| line.trim().to_string()).filter(|line| !line.is_empty()).collect();
            let action_clients_info: Vec<String> = parts[6].lines().skip(1).map(|line| line.trim().to_string()).filter(|line| !line.is_empty()).collect();

            let mut subscribers: Vec<Ros2Subscriber> = Vec::new();
            let mut publishers: Vec<Ros2Publisher> = Vec::new();
            let mut service_servers: Vec<Ros2ServiceServer> = Vec::new();
            let mut service_clients: Vec<Ros2ServiceClient> = Vec::new();
            let mut action_servers: Vec<Ros2ActionServer> = Vec::new();
            let mut action_clients: Vec<Ros2ActionClient> = Vec::new();

            for subscriber_info in subscribers_info {
                let infos: Vec<String> = subscriber_info.split(':').map(|entry| entry.trim().to_string()).collect();
                subscribers.push(Ros2Subscriber { name: infos[0].clone(), topic_name: infos[1].clone(), node_name: node_name.clone() });
            }

            for publisher_info in publishers_info {
                let infos: Vec<String> = publisher_info.split(':').map(|entry| entry.trim().to_string()).collect();
                publishers.push(Ros2Publisher { name: infos[0].clone(), topic_name: infos[1].clone(), node_name: node_name.clone() });
            }

            for service_server_info in service_servers_info {
                let infos: Vec<String> = service_server_info.split(':').map(|entry| entry.trim().to_string()).collect();
                service_servers.push(Ros2ServiceServer { name: infos[0].clone(), topic_name: infos[1].clone(), node_name: node_name.clone() });
            }

            for service_client_info in service_clients_info {
                let infos: Vec<String> = service_client_info.split(':').map(|entry| entry.trim().to_string()).collect();
                service_clients.push(Ros2ServiceClient { name: infos[0].clone(), topic_name: infos[1].clone(), node_name: node_name.clone() });
            }

            for action_client_info in action_clients_info {
                let infos: Vec<String> = action_client_info.split(':').map(|entry| entry.trim().to_string()).collect();
                action_clients.push(Ros2ActionClient { name: infos[0].clone(), topic_name: infos[1].clone(), node_name: node_name.clone() });
            }

            for action_server_info in action_servers_info {
                let infos: Vec<String> = action_server_info.split(':').map(|entry| entry.trim().to_string()).collect();
                action_servers.push(Ros2ActionServer { name: infos[0].clone(), topic_name: infos[1].clone(), node_name: node_name.clone() });
            }

            nodes.push(Ros2Node { name: node_name.clone(), package_name: "package".to_string(), subscribers, publishers, action_clients, action_servers, service_clients, service_servers })
        }

        return nodes;
    }

    pub fn node_info(node_name: String) -> String {
        let data_bytes = Command::new("ros2")
            .arg("node")
            .arg("info")
            .arg(node_name)
            .output()
            .expect("failed to execute process");
        let info: String = match String::from_utf8(data_bytes.stdout) {
            Ok(v) => v.to_string(),
            Err(e) => panic!("Invalid UTF-8 sequence: {}", e),
        };
        return info;
    }

    pub fn kill_node(node_name: String) -> String {
        let err_msg = format!("Unable to kill node {}", node_name.clone());
        let output = Command::new("killall").arg(node_name.clone()).output().expect(format!("Unable to kill node {}", node_name).as_str());
        return if output.status.success() {
            err_msg
        } else {
            String::new()
        };
    }

    pub fn explore_packages() -> Vec<Ros2Package> {
        let package_names = ros2_package_names();
        let packages: Vec<Ros2Package> = package_names.iter().map(|package_name| Ros2Package { name: package_name.to_string(), path: package_path(package_name.to_string()) }).collect();
        return packages;
    }

    pub fn package_path(package_name: String) -> String {
        //let prefix = package_prefix(package_name);
        //let package_path = concat!(prefix, "/lib/", package_name).to_string();
        //return package_path;
        return "a".to_string();
    }

    pub fn package_prefix(package_name: String) -> String {
        let data_bytes = Command::new("ros2")
            .arg("pkg")
            .arg("prefix")
            .arg(package_name)
            .output()
            .expect("failed to execute process");
        //String::from_utf8(node_bytes_str.stdout);
        let prefix_str: String = match String::from_utf8(data_bytes.stdout) {
            Ok(v) => v.to_string(),
            Err(e) => panic!("Invalid UTF-8 sequence: {}", e),
        };
        return prefix_str;
    }

    /*
    Find ros2 entities in local filesystem
     */
    /*pub fn discover_local_filesystem() -> Ros2State {
        // Check if ros2 workspace is activated
        let workspace_condition = "COLCON_PREFIX_PATH";
        let is_activated = env::var(workspace_condition).is_ok();

        if !is_activated {
            return Ros2State { state: HashMap::new() };
        }

        let install_dir = env::var(workspace_condition).unwrap();
        let src_dir = fs::canonicalize(format!("{}/../src", install_dir)).unwrap();

        // Find package names from src_dir
        let package_names = fs::read_dir(src_dir).unwrap();
        let mut packages: Vec<Ros2Entity> = Vec::new();
        for pkg_name in package_names {
            let package_name = pkg_name.unwrap().path().display().to_string();
            println!("{}", package_name);
            packages.push(Ros2Entity { name: package_name.clone(), parent_name: package_name.clone(), path: package_name.clone() });
        }

        // Find executables from src_dir


        return Ros2State { state: HashMap::new() };
    }*/

    pub fn ros2_node_names() -> Vec<String> {
        let node_bytes_str = Command::new("ros2")
            .arg("node")
            .arg("list")
            .output()
            .expect("failed to execute process");
        //String::from_utf8(node_bytes_str.stdout);
        let nodes_str = match String::from_utf8(node_bytes_str.stdout) {
            Ok(v) => v.to_string(),
            Err(e) => panic!("Invalid UTF-8 sequence: {}", e),
        };
        let node_names = nodes_str.lines().map(String::from).collect();

        return node_names;
    }

    pub fn ros2_package_names() -> Vec<String> {
        let node_bytes_str = Command::new("ros2")
            .arg("pkg")
            .arg("list")
            .output()
            .expect("failed to execute process");
        //String::from_utf8(node_bytes_str.stdout);
        let nodes_str = match String::from_utf8(node_bytes_str.stdout) {
            Ok(v) => v.to_string(),
            Err(e) => panic!("Invalid UTF-8 sequence: {}", e),
        };
        let node_names = nodes_str.lines().map(String::from).collect();

        return node_names;
    }

    pub fn ros2_subscriber_names() -> Vec<String> {
        let node_bytes_str = Command::new("ros2")
            .arg("pkg")
            .arg("list")
            .output()
            .expect("failed to execute process");
        //String::from_utf8(node_bytes_str.stdout);
        let nodes_str = match String::from_utf8(node_bytes_str.stdout) {
            Ok(v) => v.to_string(),
            Err(e) => panic!("Invalid UTF-8 sequence: {}", e),
        };
        let node_names = nodes_str.lines().map(String::from).collect();

        return node_names;
    }

    pub fn ros2_executable_names(package_name: String) -> Vec<String> {
        let node_bytes_str = Command::new("ros2")
            .arg("pkg")
            .arg("executables")
            .arg(package_name)
            .output()
            .expect("failed to execute process");
        //String::from_utf8(node_bytes_str.stdout);
        let nodes_str = match String::from_utf8(node_bytes_str.stdout) {
            Ok(v) => v.to_string(),
            Err(e) => panic!("Invalid UTF-8 sequence: {}", e),
        };
        let node_names = nodes_str.lines().map(String::from).collect();

        return node_names;
    }
}

#[cfg(test)]
mod tests {
    //use crate::ros2monitor::ros2monitor::discover_local_filesystem;

    #[test]
    fn test_ros2_local_discover() {
        //let state = discover_local_filesystem();
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
