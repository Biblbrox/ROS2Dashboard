extern crate core;

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
use ros2monitor::ros2monitor::{explore_nodes, explore_packages};
use log::{info, warn, debug, error};
use crate::ros2entites::ros2entities::{Ros2Node, Ros2State};
pub use serde_json::{json};
use tokio::net::unix::uid_t;
use tokio::time;
use crate::ros2monitor::ros2monitor::{JsonProtocol, kill_node, ros2_hot_state, ros2_state};

mod ros2entites;
mod ros2monitor;

/**
Generate json for ros2 state object
 */
fn ros2_state_json(state: Arc<Mutex<Ros2State>>) -> String {
    let state_obj: &Ros2State = &state.lock().unwrap().to_owned();

    let json_str = json!({
        "packages": state_obj.packages,
        "nodes": state_obj.nodes,
    });

    return json_str.to_string();
}

async fn handle_client(mut stream: UnixStream, current_state: Arc<Mutex<Ros2State>>) -> io::Result<()> {
    let mut buffer = [0u8; 1024];
    let nbytes = stream.read(&mut buffer[..]).await?;
    let request = str::from_utf8(&buffer).unwrap().trim().trim_matches(char::from(0));

    /// Parse request as json formatted str
    let mut parsed = JsonProtocol::new();
    match parsed.parse_struct(request) {
        Ok(()) => (),
        Err(msg) => error!("{}", msg)
    }

    let response: String = match parsed.command.as_str() {
        "state" => ros2_state_json(current_state.clone()).to_string(),
        "kill_node" => kill_node(parsed.arguments.get("node_name").unwrap().to_string()),
        _ => "Unknown request".to_string()
    };

    let msg_len: u64 = response.as_bytes().len() as u64;
    stream.write_u64(msg_len).await?; // Write message header indicates message length
    stream.write_all(&response.as_bytes()).await?; // Write the body
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
            *state_clone.lock().unwrap() = ros2_state(state_clone.clone());
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