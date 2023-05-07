import subprocess


def get_process_output(args: list):
    output = subprocess.check_output(args)
    return output
