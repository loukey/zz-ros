from domain import ConfigurableMessageUtils


class MessageOutConstructor:
    def __init__(self, config_path: str = "config/message_out_config.yaml"):
        self.configurable_message_utils = ConfigurableMessageUtils(config_path)

    def construct_message_out(self, **kwargs):
        message_out = self.configurable_message_utils.create_message_out(**kwargs)
        return self.configurable_message_utils.interpret_message(message_out)
