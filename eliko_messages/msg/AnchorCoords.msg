string anchor_id #Anchor id (unique)
string role # One letter that identifies the anchor role
string anchor_sn #24 bit identifier in 6-digit hexadecimal format (unique)
float32 x_coord # X coordinate of the anchor(in meters)
float32 y_coord # Y coordinate of the anchor(in meters)
float32 z_coord # Z coordinate of the anchor(in meters)
string last_connection_established # Timestamp when the connection to the anchor was last established (if it has never been online since the last flush of the database sends 0)
string last_connection_lost # Timestamp when the connection to the anchor was lost for the last time (if currently online sends 0)
uint8 connection_state # One digit number that shows the status of the anchor (0-No connection, 1-connected (IDs known),2-connected (role and cell size known), 3-connected and all set,-3- update mode )