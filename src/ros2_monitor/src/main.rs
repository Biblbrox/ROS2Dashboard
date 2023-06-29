extern crate core;

use std::io::{self};

use std::str;
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::{UnixStream, UnixListener};
use tokio::runtime::Runtime;
use log::{debug};
use crate::ros2entites::ros2entities::{Ros2State};
pub use serde_json::{json};
use tokio::time;
use crate::api::api::handle_request;
use crate::ros2utils::ros2utils::{ros2_state};

mod ros2entites;
mod ros2utils;
mod api;

/**
Handle client json request
 */
async fn handle_client(mut stream: UnixStream, current_state: Arc<Mutex<Ros2State>>) -> io::Result<()> {
    // Read request from stream
    let mut buffer = [0u8; 1024];
    let _nbytes = stream.read(&mut buffer[..]).await?;
    let request = str::from_utf8(&buffer).unwrap().trim().trim_matches(char::from(0));

    // Create response
    let response: String = handle_request(request.to_string(), current_state);
    debug!("Json string for response: {}", response);

    // Write back response to client
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
    let rt = Runtime::new().unwrap();
    let _guard = rt.enter();
    let listener = UnixListener::bind(socket_name)?;

    let current_state = Arc::new(Mutex::<Ros2State>::new(Ros2State::new()));

    let state_clone = current_state.clone();

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
                let _res = handle_client(stream, current_state).await;
            });
        }
    })
}