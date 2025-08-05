from ..utils import MessageEncoder, MessageDecoder


class MessageDomainService:
    def __init__(self, message_encoder: MessageEncoder, message_decoder: MessageDecoder):
        self.message_encoder = message_encoder
        self.message_decoder = message_decoder

    def encode_message(self, **message):
        m = self.message_encoder.create_message(**message)
        return self.message_encoder.interpret_message(m)

    def decode_message(self, message):
        return self.message_decoder.decode_message(message)

