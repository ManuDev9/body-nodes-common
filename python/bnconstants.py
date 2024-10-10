#
# MIT License
# 
# Copyright (c) 2024 Manuel Bottini
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


PLAYER_ALL_TAG =     "all"
PLAYER_NONE_TAG =    "none"


BODYPART_HEAD_TAG =            "head"
BODYPART_HAND_LEFT_TAG =       "hand_left"
BODYPART_LOWERARM_LEFT_TAG =   "lowerarm_left"
BODYPART_UPPERARM_LEFT_TAG =   "upperarm_left"
BODYPART_BODY_TAG =            "body"
BODYPART_LOWERARM_RIGHT_TAG =  "lowerarm_right"
BODYPART_UPPERARM_RIGHT_TAG =  "upperarm_right"
BODYPART_HAND_RIGHT_TAG =      "hand_right"
BODYPART_LOWERLEG_LEFT_TAG =   "lowerleg_left"
BODYPART_UPPERLEG_LEFT_TAG =   "upperleg_left"
BODYPART_FOOT_LEFT_TAG =       "foot_left"
BODYPART_LOWERLEG_RIGHT_TAG =  "lowerleg_right"
BODYPART_UPPERLEG_RIGHT_TAG =  "upperleg_right"
BODYPART_FOOT_RIGHT_TAG =      "foot_right"
BODYPART_UPPERBODY_TAG =       "upperbody"
BODYPART_LOWERBODY_TAG =       "lowerbody"
BODYPART_KATANA_TAG =          "katana"
BODYPART_UNTAGGED_TAG =        "untagged"
BODYPART_ALL_TAG =             "all"

# ACTIONS
ACTION_TYPE_TAG =              "type"
ACTION_PLAYER_TAG =            "player"
ACTION_BODYPART_TAG =          "bodypart"

ACTION_TYPE_NONE_TAG =         "none"
ACTION_TYPE_HAPTIC_TAG =       "haptic"
ACTION_TYPE_SETPLAYER_TAG =    "set_player"
ACTION_TYPE_SETBODYPART_TAG =  "set_bodypart"
ACTION_TYPE_ENABLESENSOR_TAG = "enable_sensor"
ACTION_TYPE_SETWIFI_TAG =      "set_wifi"

ACTION_HAPTIC_DURATION_MS_TAG =      "duration_ms"
ACTION_HAPTIC_STRENGTH_TAG =         "strength"
ACTION_SETPLAYER_NEWPLAYER_TAG =     "new_player"
ACTION_SETBODYPART_NEWBODYPART_TAG = "new_bodypart"
ACTION_ENABLESENSOR_SENSORTYPE_TAG = "sensortype"
ACTION_ENABLESENSOR_ENABLE_TAG =     "enable"
ACTION_SETWIFI_SSID_TAG =            "ssid"
ACTION_SETWIFI_PASSWORD_TAG =        "password"
ACTION_SETWIFI_MULTICASTMESSAGE_TAG ="multicast_message"

# MESSAGE
MESSAGE_PLAYER_TAG =        "player"
MESSAGE_BODYPART_TAG =      "bodypart"
MESSAGE_SENSORTYPE_TAG =    "sensortype"
MESSAGE_VALUE_TAG =         "value"

# MEMORY
MEMORY_BODYPART_TAG =              "bodypart"
MEMORY_BODYPART_GLOVE_TAG =        "bodypart_glove"
MEMORY_BODYPART_SHOE_TAG =         "bodypart_shoe"
MEMORY_PLAYER_TAG =                "player"
MEMORY_WIFI_SSID_TAG =             "wifi_ssid"
MEMORY_WIFI_PASSWORD_TAG =         "wifi_password"
MEMORY_WIFI_MULTICASTMESSAGE_TAG = "multicast_message"

# SENSOR DATA TYPES 
SENSORTYPE_NONE_TAG =                   "none"
SENSORTYPE_ORIENTATION_ABS_TAG =        "orientation_abs"
SENSORTYPE_ACCELERATION_REL_TAG =       "acceleration_rel"
SENSORTYPE_GLOVE_TAG =                  "glove"
SENSORTYPE_SHOE_TAG =                   "shoe"
SENSORTYPE_ANGULARVELOCITY_REL_TAG =    "angularvelocity_rel"

# SENSOR STATUS
SENSOR_STATUS_NOT_ACCESSIBLE =  1
SENSOR_STATUS_CALIBRATING =     2
SENSOR_STATUS_WORKING =         3

# CONNECTION STATUS
CONNECTION_STATUS_NOT_CONNECTED = 0
CONNECTION_STATUS_WAITING_ACK =   1
CONNECTION_STATUS_CONNECTED =     2


# Wifi Connections
BODYNODES_PORT =                    12345
BODYNODES_MULTICAST_PORT =          12346
BODYNODES_WIFI_SSID_DEFAULT =       "BodynodeHotspot"
BODYNODES_WIFI_PASS_DEFAULT =       "bodynodes1"
BODYNODES_MULTICASTGROUP_DEFAULT =   "239.192.1.99"
BODYNODES_MULTICASTMESSAGE_DEFAULT = "BN"

# Bluetooth Connections
#TODO

# BLE Connections
BLE_BODYNODES_NAME                                  = "Bodynode"
BLE_BODYNODES_SERVICE_UUID                          = "0000CCA0-0000-1000-8000-00805F9B34FB"
BLE_BODYNODES_CHARA_PLAYER_UUID                     = "0000CCA1-0000-1000-8000-00805F9B34FB"
BLE_BODYNODES_CHARA_BODYPART_UUID                   = "0000CCA2-0000-1000-8000-00805F9B34FB"
BLE_BODYNODES_CHARA_ORIENTATION_ABS_VALUE_UUID      = "0000CCA3-0000-1000-8000-00805F9B34FB"
BLE_BODYNODES_CHARA_ACCELERATION_REL_VALUE_UUID     = "0000CCA4-0000-1000-8000-00805F9B34FB"
BLE_BODYNODES_CHARA_GLOVE_VALUE_UUID                = "0000CCA5-0000-1000-8000-00805F9B34FB"
BLE_BODYNODES_CHARA_SHOE_UUID                       = "0000CCA6-0000-1000-8000-00805F9B34FB"
BLE_BODYNODES_CHARA_ANGULARVELOCITY_REL_VALUE_UUID  = "0000CCA7-0000-1000-8000-00805F9B34FB"