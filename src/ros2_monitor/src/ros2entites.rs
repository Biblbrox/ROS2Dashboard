pub mod ros2entities {
    use std::string::String;
    use serde::{Deserialize, Serialize};


    #[derive(Deserialize, Clone)]
    pub struct Ros2State {
        pub packages: Vec<Ros2Package>,
        pub executables: Vec<Ros2Executable>,
        pub nodes: Vec<Ros2Node>,
        pub topics: Vec<Ros2Topic>,
    }

    impl Ros2State {
        pub fn new() -> Ros2State {
            return Ros2State {
                packages: Vec::new(),
                executables: Vec::new(),
                nodes: Vec::new(),
                topics: Vec::new(),
            };
        }
    }


    #[derive(Serialize, Deserialize, Clone)]
    pub struct Ros2Package {
        pub name: String,
        pub path: String,
    }

    #[derive(Serialize, Deserialize, Clone)]
    pub struct Ros2Executable {
        pub name: String,
        pub package_name: String,
    }

    #[derive(Serialize, Deserialize, Clone)]
    pub struct Ros2Node {
        pub name: String,
        pub package_name: String,
        pub subscribers: Vec<Ros2Subscriber>,
        pub publishers: Vec<Ros2Publisher>,
        pub service_servers: Vec<Ros2ServiceServer>,
        pub service_clients: Vec<Ros2ServiceClient>,
        pub action_servers: Vec<Ros2ActionServer>,
        pub action_clients: Vec<Ros2ActionClient>,
    }

    #[derive(Serialize, Deserialize, Clone)]
    pub struct Ros2Subscriber {
        pub name: String,
        pub node_name: String,
        pub topic_name: String,
    }

    #[derive(Serialize, Deserialize, Clone)]
    pub struct Ros2Publisher {
        pub name: String,
        pub node_name: String,
        pub topic_name: String,
    }

    #[derive(Serialize, Deserialize, Clone)]
    pub struct Ros2ActionClient {
        pub name: String,
        pub node_name: String,
        pub topic_name: String,
    }

    #[derive(Serialize, Deserialize, Clone)]
    pub struct Ros2ActionServer {
        pub name: String,
        pub node_name: String,
        pub topic_name: String,
    }

    #[derive(Serialize, Deserialize, Clone)]
    pub struct Ros2ServiceServer {
        pub name: String,
        pub node_name: String,
        pub topic_name: String,
    }

    #[derive(Serialize, Deserialize, Clone)]
    pub struct Ros2ServiceClient {
        pub name: String,
        pub node_name: String,
        pub topic_name: String,
    }

    /// I will have done with these entities later...
    /// ....
    #[derive(Serialize, Deserialize, Clone)]
    pub struct Ros2Client {
        pub name: String,
        pub node_name: String,
    }

    #[derive(Serialize, Deserialize, Clone)]
    pub struct Ros2Topic {
        pub name: String,
        pub node_name: String,
        pub topic_type: String
    }
}