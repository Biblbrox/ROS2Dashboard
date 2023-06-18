pub mod ros2monitor {
    use std::collections::HashMap;
    use std::process::Command;
    use std::string::String;

    use rclrs::Context;
    use std::{env, fs};
    use std::path::PathBuf;
    use crate::ros2entites::ros2entities::{Ros2Entity, Ros2State, build_state, Ros2StateList, Ros2Publisher, Ros2Subscriber};

    pub fn init_ros2() -> Context {
        let context = rclrs::Context::new(env::args());
        return context.unwrap();
    }

    pub fn ros2_state() -> Ros2State {
        let nodes: Vec<Ros2Entity> = nodes();
        let packages: Vec<Ros2Entity> = packages();
        //let publishers: Vec<Ros2Entity> = publishers();
        //let subscribers: Vec<Ros2Entity> = subscribers();
        let mut state_list: HashMap<String, Vec<Ros2Entity>> = HashMap::new();
        state_list.insert("nodes".to_string(), nodes);
        state_list.insert("packages".to_string(), packages);
        //state_list.insert("publishers".to_string(), publishers);
        //state_list.insert("subscribers".to_string(), subscribers);
        let state = build_state(Ros2StateList { state: state_list });

        return state;
    }

    pub fn nodes() -> Vec<Ros2Entity> {
        let node_names = ros2_node_names();
        let nodes: Vec<Ros2Entity> = node_names.iter().map(|node_name| Ros2Entity { parent_name: node_name.to_string(), name: node_name.to_string(), path: "".to_string() }).collect();
        return nodes;
    }

    pub fn packages() -> Vec<Ros2Entity> {
        let package_names = ros2_package_names();
        let packages: Vec<Ros2Entity> = package_names.iter().map(|package_name| Ros2Entity { parent_name: package_name.to_string(), name: package_name.to_string(), path: "".to_string() }).collect();
        return packages;
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
