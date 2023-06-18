use std::collections::HashMap;
use std::io::{self, Read, Write};
use std::ops::Deref;
use std::str;
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::{UnixStream, UnixListener};
use tokio::runtime::Runtime;
use ros2monitor::ros2monitor::{nodes, packages};
use log::{info, warn, debug};
use crate::ros2entites::ros2entities::{build_state, EntryType, Ros2Entity, Ros2Node, Ros2Service, Ros2State, Ros2Topic};
pub use serde_json::{json};
use tokio::net::unix::uid_t;
use tokio::time;

mod ros2entites;
mod ros2monitor;


/**
Possible commands:
list_nodes ->
 */

/*fn get_node_list() -> Vec<Ros2Node> {

}*/

/**
Generate json for ros2 state object
 */
fn ros2_state_json(state: Arc<Mutex<Ros2State>>) -> String {
    let state_obj: &Ros2State = &state.lock().unwrap().to_owned();
    // Build json
    let json_str = json!({
        "packages": state_obj.packages,
        "nodes": state_obj.nodes,
        "subscribers": state_obj.subscribers,
        "publishers": state_obj.publishers
    });

    return json_str.to_string();
}

async fn handle_client(mut stream: UnixStream, current_state: Arc<Mutex<Ros2State>>) -> io::Result<()> {
    let mut buffer = [0u8; 1024];
    let nbytes = stream.read(&mut buffer[..]).await?;
    let mut line: String = str::from_utf8(&buffer).unwrap().trim().to_string();
    line = line.trim_matches(char::from(0)).parse().unwrap();
    let mut response: String;
    if line.eq("node_list") {
        debug!("Node list request");
        response = "Response".to_string();
    } else if line.eq("is_new_entities") {
        debug!("New entities request");
        response = "true".to_string();
    } else if line.eq("state") {
        debug!("State request");
        response = ros2_state_json(current_state.clone()).to_string();
    } else {
        response = format!("Unknown request: {}", line);
    }
    let msg_len: u64 = response.as_bytes().len() as u64;
    stream.write_u64(msg_len).await?; // Write message header indicated message length
    stream.write_all(&response.as_bytes()).await?; // Write the rest
    Ok(())
}

fn main() -> io::Result<()> {
    simple_logger::init_with_level(log::Level::Debug).unwrap();

    let socket_name = "/tmp/ros2monitor.sock";
    if Path::new(socket_name).exists() {
        std::fs::remove_file(socket_name).expect("Unable to release socket");
    }
    let mut rt = Runtime::new().unwrap();
    let guard = rt.enter();
    let listener = UnixListener::bind(socket_name)?;

    let mut current_state = Arc::new(Mutex::<Ros2State>::new(Ros2State::new()));

    let mut state_clone = current_state.clone();

    let update_ros2_state = rt.spawn(async move {
        let mut interval = time::interval(Duration::from_millis(10000));
        loop {
            interval.tick().await;
            debug!("Every minute at 00'th and 30'th second");
            let ros2_packages = packages();
            let ros2_nodes = nodes();
            let mut ros2_state_list = ros2entites::ros2entities::Ros2StateList {
                state: HashMap::new()
            };
            ros2_state_list.state.insert("packages".to_string(), ros2_packages);
            ros2_state_list.state.insert("nodes".to_string(), ros2_nodes);
            let new_state = build_state(ros2_state_list);
            *state_clone.lock().unwrap() = new_state;
        }
    });

    rt.spawn(update_ros2_state);

    // Accept connections from clients
    rt.block_on(async {
        loop {
            let (stream, _) = listener.accept().await?;
            let current_state = current_state.clone();
            tokio::spawn(async move {
                let res = handle_client(stream, current_state).await;
            });
        }
    })
}