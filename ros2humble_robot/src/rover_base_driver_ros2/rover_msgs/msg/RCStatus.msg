# This message contains the status of the remote

std_msgs/Header header              # Header stamp should be acquisition time of measure.
bool connect                        # Mark if the remote is connected
int8[10] channel                    # Remote control channel ratio value