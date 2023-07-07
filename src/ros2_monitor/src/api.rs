pub mod api {
    use std::sync::{Arc, Mutex};
    use log::{error, warn};
    use serde_json::json;
    use crate::ros2entites::ros2entities::{Ros2State, Ros2Topic};
    use crate::ros2utils::ros2utils::{is_node_running, JsonProtocol, kill_node, run_node};

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
            "start_node" => "".to_string(),
            "rename_topic" => "".to_string(),
            _ => "Unknown request".to_string()
        };

        return response;
    }

    /// This function renames topic to another name. In order to do this topic's node need to be
    /// restarted. Due this rename_topic_command works well for lifecycle nodes. It can be used with
    /// regular nodes without any guarantee of success. It returns json string contains if command
    /// was completed with success.
    /// # Arguments
    ///
    /// * `request`: request data
    /// * `_current_state`: current info about ros2 entities
    ///
    /// returns: String
    ///
    /// # Examples
    ///
    /// ```
    ///
    /// ```
    pub fn rename_topic_command(request: &JsonProtocol, current_state: Arc<Mutex<Ros2State>>) -> String {
        let new_topic_name = if request.arguments.contains_key("new_topic_name") {
            request.arguments.get("node_name").unwrap().to_string()
        } else {
            r#"{"result": "failure", "msg": "You must provide new_topic_name argument for command rename_topic"}"#.to_string()
        };

        let old_topic_name = if request.arguments.contains_key("old_topic_name") {
            request.arguments.get("node_name").unwrap().to_string()
        } else {
            r#"{"result": "failure", "msg": "You must provide old_topic_name argument for command rename_topic"}"#.to_string()
        };

        // Find node name for topic
        let topic: Ros2Topic = current_state.lock().unwrap().topics.iter().find(|&topic| topic.name == new_topic_name).unwrap().clone();
        let node_name = topic.node_name;
        // Check if node is running. If yes then kill it
        if is_node_running(node_name.clone()) {
            kill_node(node_name.clone());
        }

        // Restart node with altered topic name
        let res = run_node(node_name.clone(), ["--ros-args", "-r", old_topic_name.as_str(), new_topic_name.as_str()]);
        return res;
    }

    pub fn start_node_command(request: &JsonProtocol, _current_state: Arc<Mutex<Ros2State>>) {}

    pub fn state_command(_request: &JsonProtocol, current_state: Arc<Mutex<Ros2State>>) -> String {
        return ros2_state_json(current_state.clone());
    }

    pub fn kill_node_command(request: &JsonProtocol, _current_state: Arc<Mutex<Ros2State>>) -> String {
        // Extract node name from request
        let node_name = if request.arguments.contains_key("node_name") {
            request.arguments.get("node_name").unwrap().to_string()
        } else {
            r#"{"result": "failure", "msg": "You must provide node_name argument for command kill_node"}"#.to_string()
        };

        if node_name.is_empty() {
            warn!("Node {} doesn't running. So, it is impossible to be killed", node_name);
            return "".to_string();
        }

        return kill_node(node_name);
    }
}