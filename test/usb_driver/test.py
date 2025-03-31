from utils.command_utils import format_command

hex_command = format_command(joint_angles=[0.0] * 6, control=0x01, mode=0x08, encoding='hex')
string_command = format_command(joint_angles=[0.0] * 6, control=0x01, mode=0x08, encoding='string')

print(hex_command)
print(string_command)

print(len(hex_command))
