from ...utils import MessageEncoder, MessageDecoder


class MessageDomainService:
    """消息领域服务。
    
    负责消息的编码和解码，作为 MessageEncoder 和 MessageDecoder 的门面。
    
    Attributes:
        message_encoder (MessageEncoder): 消息编码器。
        message_decoder (MessageDecoder): 消息解码器。
    """
    
    def __init__(self, message_encoder: MessageEncoder, message_decoder: MessageDecoder):
        """初始化消息领域服务。
        
        Args:
            message_encoder (MessageEncoder): 注入的消息编码器。
            message_decoder (MessageDecoder): 注入的消息解码器。
        """
        self.message_encoder = message_encoder
        self.message_decoder = message_decoder

    def encode_message(self, **message: Any) -> str:
        """编码消息。
        
        Args:
            **message: 消息字段键值对。
            
        Returns:
            str: 编码后的十六进制消息字符串。
        """
        m = self.message_encoder.create_message(**message)
        return self.message_encoder.interpret_message(m)

    def decode_message(self, message: str) -> Any:
        """解码消息。
        
        Args:
            message (str): 十六进制消息字符串。
            
        Returns:
            Any: 解码后的消息对象 (MessageIn)。
        """
        return self.message_decoder.decode_message(message)

