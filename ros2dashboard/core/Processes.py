import subprocess


def get_process_output(args: list):
    # output = subprocess.call(args)
    output = subprocess.check_output(args)
    return output
