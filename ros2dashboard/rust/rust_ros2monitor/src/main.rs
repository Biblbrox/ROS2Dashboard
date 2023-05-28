use rust_ros2monitor_lib::ros2_monitor::ros2_node_names;
use rust_ros2monitor_lib::ros2_monitor::ros2_package_names;

fn main() {
    let node_names = ros2_node_names();
    println!("Nodes:");
    for node_name in node_names {
        println!("{node_name}");
    }

    println!("******************************************");

    let package_names = ros2_package_names();
    println!("Packages:");
    for package_name in package_names {
        println!("{package_name}");
    }
}
