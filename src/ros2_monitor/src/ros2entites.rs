pub mod ros2entities {
    use std::collections::hash_map::Entry;
    use std::collections::HashMap;
    use std::os::linux::raw::stat;
    use std::string::String;
    use serde::{Deserialize, Serialize, Serializer};
    use serde_json::Result;

    pub type EntryType = Vec<Ros2Entity>;

    #[derive(Deserialize, Clone)]
    pub struct Ros2State {
        pub packages: Vec<Ros2Package>,
        pub nodes: Vec<Ros2Node>,
        pub executables: Vec<Ros2Executable>,
        pub topics: Vec<Ros2Topic>,
        pub subscribers: Vec<Ros2Subscriber>,
        pub publishers: Vec<Ros2Publisher>,
        pub action_clients: Vec<Ros2ActionClient>,
        pub action_servers: Vec<Ros2ActionServer>,
        pub clients: Vec<Ros2Client>,
    }

    impl Ros2State {
        pub fn new() -> Ros2State {
            return Ros2State {
                packages: Vec::new(),
                nodes: Vec::new(),
                executables: Vec::new(),
                topics: Vec::new(),
                subscribers: Vec::new(),
                publishers: Vec::new(),
                action_clients: Vec::new(),
                action_servers: Vec::new(),
                clients: Vec::new(),
            };
        }
    }

    #[derive(Serialize, Deserialize, Clone)]
    pub struct Ros2StateList {
        pub state: HashMap<String, EntryType>,
    }


    pub fn build_state(data: Ros2StateList) -> Ros2State {
        let mut packages: Vec<Ros2Package> = Vec::new();
        let mut nodes: Vec<Ros2Node> = Vec::new();
        let mut clients: Vec<Ros2Client> = Vec::new();
        let mut executables: Vec<Ros2Executable> = Vec::new();
        let mut topics: Vec<Ros2Topic> = Vec::new();
        let mut services: Vec<Ros2Service> = Vec::new();
        let mut subscribers: Vec<Ros2Subscriber> = Vec::new();
        let mut publishners: Vec<Ros2Publisher> = Vec::new();
        let mut action_clients: Vec<Ros2ActionClient> = Vec::new();
        let mut action_servers: Vec<Ros2ActionServer> = Vec::new();

        if data.state.contains_key("packages") {
            let packages_data: Vec<Ros2Entity> = data.state.get("packages").unwrap().to_vec();
            for entry in packages_data {
                packages.push(Ros2Package { name: entry.name, path: "".to_string() });
            }
        }

        if data.state.contains_key("executables") {
            let executables_data: Vec<Ros2Entity> = data.state.get("executables").unwrap().to_vec();
            for executable in executables_data {
                executables.push(Ros2Executable { name: executable.name, package_name: "".to_string() })
            }
        }

        if data.state.contains_key("nodes") {
            let nodes_data: Vec<Ros2Entity> = data.state.get("nodes").unwrap().to_vec();
            for node in nodes_data {
                nodes.push(Ros2Node { name: node.name, package_name: "".to_string() });
            }
        }

        if data.state.contains_key("topics") {
            let topics_data = data.state.get("topics").unwrap().to_vec();

            for topic in topics_data {
                topics.push(Ros2Topic { name: topic.name, node_name: "".to_string() });
            }
        }

        if data.state.contains_key("services") {
            let services_data = data.state.get("services").unwrap().to_vec();

            for service in services_data {
                services.push(Ros2Service { name: service.name, node_name: "".to_string() })
            }
        }

        if data.state.contains_key("publishers") {
            let publishers_data = data.state.get("publishers").unwrap().to_vec();

            for publisher in publishers_data {
                publishners.push(Ros2Publisher { name: publisher.name, node_name: "".to_string() });
            }
        }

        if data.state.contains_key("subscribers") {
            let subscribers_data = data.state.get("subscribers").unwrap().to_vec();

            for subscriber in subscribers_data {
                subscribers.push(Ros2Subscriber { name: subscriber.name, node_name: "".to_string() });
            }
        }

        if data.state.contains_key("clients") {
            let clients_data = data.state.get("clients").unwrap().to_vec();

            for client in clients_data {
                clients.push(Ros2Client { name: client.name, node_name: "".to_string() });
            }
        }

        if data.state.contains_key("action_clients") {
            let action_clients_data = data.state.get("action_clients").unwrap().to_vec();

            for action_client in action_clients_data {
                action_clients.push(Ros2ActionClient { name: action_client.name, node_name: "".to_string() });
            }
        }

        if data.state.contains_key("action_servers") {
            let action_servers_data = data.state.get("action_servers").unwrap().to_vec();

            for action_server in action_servers_data {
                action_servers.push(Ros2ActionServer { name: action_server.name, node_name: "".to_string() });
            }
        }

        return Ros2State {
            nodes: nodes,
            action_clients: action_clients,
            action_servers: action_servers,
            packages: packages,
            executables: executables,
            topics: topics,
            subscribers: subscribers,
            publishers: publishners,
            clients: clients,
        };
    }

    #[derive(Clone, Serialize, Deserialize)]
    pub struct Ros2Entity {
        pub name: String,
        pub parent_name: String,
        pub path: String, // Actual for packages and executables
    }

    impl ToString for Ros2Entity {
        // Required method
        fn to_string(&self) -> String {
            return format!("{}:{}", self.parent_name, self.name);
        }
    }

    #[derive(Serialize, Deserialize, Clone)]
    pub struct Ros2Node {
        name: String,
        package_name: String,
    }

    #[derive(Serialize, Deserialize, Clone)]
    pub struct Ros2Package {
        name: String,
        path: String,
    }

    #[derive(Serialize, Deserialize, Clone)]
    pub struct Ros2Executable {
        name: String,
        package_name: String,
    }

    #[derive(Serialize, Deserialize, Clone)]
    pub struct Ros2Topic {
        name: String,
        node_name: String,
    }

    #[derive(Serialize, Deserialize, Clone)]
    pub struct Ros2Subscriber {
        name: String,
        node_name: String,
    }

    #[derive(Serialize, Deserialize, Clone)]
    pub struct Ros2Publisher {
        name: String,
        node_name: String,
    }

    #[derive(Serialize, Deserialize, Clone)]
    pub struct Ros2Client {
        name: String,
        node_name: String,
    }

    #[derive(Serialize, Deserialize, Clone)]
    pub struct Ros2ActionClient {
        name: String,
        node_name: String,
    }

    #[derive(Serialize, Deserialize, Clone)]
    pub struct Client {
        name: String,
        node_name: String,
    }

    #[derive(Serialize, Deserialize, Clone)]
    pub struct Ros2ActionServer {
        name: String,
        node_name: String,
    }

    #[derive(Serialize, Deserialize, Clone)]
    pub struct Ros2Service {
        name: String,
        node_name: String,
    }
}