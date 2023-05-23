use pyo3::prelude::*;

pub mod ros2_monitor {
    use std::process::Command;
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

/// Formats the sum of two numbers as string.
#[pyfunction]
fn sum_as_string(a: usize, b: usize) -> PyResult<String> {
    Ok((a + b).to_string())
}

#[pyfunction]
fn ros2_package_names_r() -> PyResult<Vec<String>> {
    return Ok(ros2_monitor::ros2_package_names());
}

#[pyfunction]
fn ros2_node_names_r() -> PyResult<Vec<String>> {
    return Ok(ros2_monitor::ros2_node_names());
}

#[pyfunction]
fn ros2_executable_names_r(pkg_name: String) -> PyResult<Vec<String>> {
    return Ok(ros2_monitor::ros2_executable_names(pkg_name));
}

/// A Python module implemented in Rust.
#[pymodule]
fn rust_ros2monitor(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(sum_as_string, m)?)?;
    m.add_function(wrap_pyfunction!(ros2_package_names_r, m)?)?;
    m.add_function(wrap_pyfunction!(ros2_node_names_r, m)?)?;
    m.add_function(wrap_pyfunction!(ros2_executable_names_r, m)?)?;
    Ok(())
}
