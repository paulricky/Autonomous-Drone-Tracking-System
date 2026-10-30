# Read-only inventory. None of these fields authorize physical control.
# Wire framing is MSP v1; protocol_version is the API reply's protocol byte,
# not a statement that MSP v2 framing or control interfaces were verified.
builtin_interfaces/Time stamp
bool api_available
uint8 protocol_version
uint8 api_major
uint8 api_minor
string firmware_variant
uint8[] firmware_version_raw
uint8[] board_info_raw
string board_identifier
bool control_interface_verified
bool landing_interface_verified
string detail
