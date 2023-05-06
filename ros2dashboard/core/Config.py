import configparser


class Config:
    def __init__(self, path):
        self.path = path
        self.config = configparser.ConfigParser()

    def load(self):
        self.config.read(self.path)

    def save(self):
        with open(self.path, 'w') as config_file:
            self.config.write(config_file)

    def get_value(self, section, key, default=None, expected_type=None):
        if expected_type is None:
            return self.config.get(section, key, fallback=default)
        else:
            return self._get_typed_value(section, key, default, expected_type)

    def set_value(self, section, key, value):
        if isinstance(value, str):
            self.config.set(section, key, value)
        else:
            self.config.set(section, key, str(value))

    def _get_typed_value(self, section, key, default, expected_type):
        try:
            if expected_type == bool:
                return self.config.getboolean(section, key)
            elif expected_type == int:
                return self.config.getint(section, key)
            elif expected_type == float:
                return self.config.getfloat(section, key)
            elif expected_type == list:
                raw_value = self.config.get(section, key, fallback=default)
                if raw_value:
                    return [x.strip() for x in raw_value.split(',')]
                else:
                    return []
            elif expected_type == tuple:
                raw_value = self.config.get(section, key, fallback=default)
                if raw_value:
                    return tuple([x.strip() for x in raw_value.split(',')])
                else:
                    return ()
            elif expected_type == dict:
                raw_value = self.config.get(section, key, fallback=default)
                if raw_value:
                    return dict([x.strip().split(':') for x in raw_value.split(',')])
                else:
                    return {}
            else:
                raise ValueError(f'Unsupported expected type: {expected_type}')
        except (configparser.NoSectionError, configparser.NoOptionError):
            return default
