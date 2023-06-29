pub mod api {
    use std::sync::{Arc, Mutex};
    use log::{error, warn};
    use serde_json::json;
    use crate::ros2entites::ros2entities::Ros2State;
    use crate::ros2utils::ros2utils::{JsonProtocol, kill_node};

    /**
    Generate json for ros2 state object
     */
    pub fn ros2_state_json(state: Arc<Mutex<Ros2State>>) -> String {
        let state_obj: &Ros2State = &state.lock().unwrap().to_owned();

        let json_str = json!({
            "packages": state_obj.packages,
            "nodes": state_obj.nodes,
            "topics": state_obj.topics
        });

        return json_str.to_string();
    }

    pub fn handle_request(request: String, current_state: Arc<Mutex<Ros2State>>) -> String {
        // Parse request as json formatted str
        let mut parsed = JsonProtocol::new();
        match parsed.parse_request(request.as_str()) {
            Ok(()) => (),
            Err(msg) => error!("{}", msg)
        }
        let command = parsed.command.clone();
        let response: String = match command.as_str() {
            "state" => state_command(&parsed, current_state),
            "kill_node" => kill_node_command(&parsed, current_state),
            _ => "Unknown request".to_string()
        };

        return response;
    }

    pub fn state_command(_request: &JsonProtocol, current_state: Arc<Mutex<Ros2State>>) -> String {
        return ros2_state_json(current_state.clone());
    }

    pub fn kill_node_command(request: &JsonProtocol, _current_state: Arc<Mutex<Ros2State>>) -> String {
        // Extract node name from request
        let node_name = if request.arguments.contains_key("node_name") {
            request.arguments.get("node_name").unwrap().to_string()
        } else {
            "".to_string()
        };

        if node_name.is_empty() {
            warn!("Node {} doesn't running. So, it is impossible to be killed", node_name);
            return "".to_string();
        }

        return kill_node(node_name);
    }
}