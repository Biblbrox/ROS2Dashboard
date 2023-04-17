import datetime
import logging as inner_logging

class CustomFormatter(inner_logging.Formatter):
    """Logging colored formatter, adapted from https://stackoverflow.com/a/56944256/3638629"""

    grey = '\x1b[38;21m'
    blue = '\x1b[38;5;39m'
    yellow = '\x1b[38;5;226m'
    red = '\x1b[38;5;196m'
    bold_red = '\x1b[31;1m'
    reset = '\x1b[0m'

    def __init__(self, fmt):
        super().__init__()
        self.fmt = fmt
        self.FORMATS = {
            inner_logging.DEBUG: self.grey + self.fmt + self.reset,
            inner_logging.INFO: self.blue + self.fmt + self.reset,
            inner_logging.WARNING: self.yellow + self.fmt + self.reset,
            inner_logging.ERROR: self.red + self.fmt + self.reset,
            inner_logging.CRITICAL: self.bold_red + self.fmt + self.reset
        }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = inner_logging.Formatter(log_fmt)
        return formatter.format(record)


def init_logger():
    # Create custom logger logging all five levels
    logger = inner_logging.getLogger(__name__)
    logger.setLevel(inner_logging.DEBUG)

    # Define format for logs
    fmt = '%(asctime)s | %(levelname)8s | %(message)s'

    # Create stdout handler for logging to the console (logs all five levels)
    stdout_handler = inner_logging.StreamHandler()
    stdout_handler.setLevel(inner_logging.DEBUG)
    stdout_handler.setFormatter(CustomFormatter(fmt))

    # Create file handler for logging to a file (logs all five levels)
    today = datetime.date.today()
    file_handler = inner_logging.FileHandler('my_app_{}.log'.format(today.strftime('%Y_%m_%d')))
    file_handler.setLevel(inner_logging.DEBUG)
    file_handler.setFormatter(inner_logging.Formatter(fmt))

    # Add both handlers to the logger
    logger.addHandler(stdout_handler)
    logger.addHandler(file_handler)

    return logger

logging = init_logger()