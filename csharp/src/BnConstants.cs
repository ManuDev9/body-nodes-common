/**
* MIT License
* 
* Copyright (c) 2019-2025 Manuel Bottini
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:

* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.

* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

namespace BodynodesDev.Common
{
    public class BnConstants
    {

        private BnConstants()
        {
            throw new NotSupportedException("This is a static class. It cannot be instantiated.");
        }

        // PLAYER NAMES CASES
        public readonly static string PLAYER_ALL_TAG =     "all";
        public readonly static string PLAYER_NONE_TAG =    "none";

        // BODYPARTS
        public readonly static string BODYPART_NONE_TAG =            "none";
        public readonly static string BODYPART_HEAD_TAG =            "head";
        public readonly static string BODYPART_HAND_LEFT_TAG =       "hand_left";
        public readonly static string BODYPART_LOWERARM_LEFT_TAG =   "lowerarm_left";
        public readonly static string BODYPART_UPPERARM_LEFT_TAG =   "upperarm_left";
        public readonly static string BODYPART_BODY_TAG =            "body";
        public readonly static string BODYPART_LOWERARM_RIGHT_TAG =  "lowerarm_right";
        public readonly static string BODYPART_UPPERARM_RIGHT_TAG =  "upperarm_right";
        public readonly static string BODYPART_HAND_RIGHT_TAG =      "hand_right";
        public readonly static string BODYPART_LOWERLEG_LEFT_TAG =   "lowerleg_left";
        public readonly static string BODYPART_UPPERLEG_LEFT_TAG =   "upperleg_left";
        public readonly static string BODYPART_FOOT_LEFT_TAG =       "foot_left";
        public readonly static string BODYPART_LOWERLEG_RIGHT_TAG =  "lowerleg_right";
        public readonly static string BODYPART_UPPERLEG_RIGHT_TAG =  "upperleg_right";
        public readonly static string BODYPART_FOOT_RIGHT_TAG =      "foot_right";
        public readonly static string BODYPART_UPPERBODY_TAG =       "upperbody";
        public readonly static string BODYPART_LOWERBODY_TAG =       "lowerbody";
        public readonly static string BODYPART_KATANA_TAG =          "katana";
        public readonly static string BODYPART_UNTAGGED_TAG =        "untagged";
        public readonly static string BODYPART_ALL_TAG =             "all";

        // ACTIONS
        public readonly static string ACTION_TYPE_TAG =              "type";
        public readonly static string ACTION_PLAYER_TAG =            "player";
        public readonly static string ACTION_BODYPART_TAG =          "bodypart";

        public readonly static string ACTION_TYPE_NONE_TAG =         "none";
        public readonly static string ACTION_TYPE_HAPTIC_TAG =       "haptic";
        public readonly static string ACTION_TYPE_SETPLAYER_TAG =    "set_player";
        public readonly static string ACTION_TYPE_SETBODYPART_TAG =  "set_bodypart";
        public readonly static string ACTION_TYPE_ENABLESENSOR_TAG = "enable_sensor";
        public readonly static string ACTION_TYPE_SETWIFI_TAG =      "set_wifi";

        public readonly static string ACTION_HAPTIC_DURATION_MS_TAG =          "duration_ms";
        public readonly static string ACTION_HAPTIC_STRENGTH_TAG =             "strength";
        public readonly static string ACTION_SETPLAYER_NEWPLAYER_TAG =         "new_player";
        public readonly static string ACTION_SETBODYPART_NEWBODYPART_TAG =     "new_bodypart";
        public readonly static string ACTION_ENABLESENSOR_SENSORTYPE_TAG =     "sensortype";
        public readonly static string ACTION_ENABLESENSOR_ENABLE_TAG =         "enable";
        public readonly static string ACTION_SETWIFI_SSID_TAG =                "ssid";
        public readonly static string ACTION_SETWIFI_PASSWORD_TAG =            "password";
        public readonly static string ACTION_SETWIFI_MULTICASTMESSAGE_TAG =    "multicast_message";

        // MESSAGE
        public readonly static string MESSAGE_PLAYER_TAG =        "player";
        public readonly static string MESSAGE_BODYPART_TAG =      "bodypart";
        public readonly static string MESSAGE_SENSORTYPE_TAG =    "sensortype";
        public readonly static string MESSAGE_VALUE_TAG =         "value";

        // GLOVE
        public readonly static uint GLOVE_ANGLE_MIGNOLO_INDEX =  0;
        public readonly static uint GLOVE_ANGLE_ANULARE_INDEX =  1;
        public readonly static uint GLOVE_ANGLE_MEDIO_INDEX =    2;
        public readonly static uint GLOVE_ANGLE_INDICE_INDEX =   3;
        public readonly static uint GLOVE_ANGLE_POLLICE_INDEX =  4;
        public readonly static uint GLOVE_TOUCH_MIGNOLO_INDEX =  5;
        public readonly static uint GLOVE_TOUCH_ANULARE_INDEX =  6;
        public readonly static uint GLOVE_TOUCH_MEDIO_INDEX =    7;
        public readonly static uint GLOVE_TOUCH_INDICE_INDEX =   8;

        // MEMORY
        public readonly static string MEMORY_BODYPART_TAG =              "bodypart";
        public readonly static string MEMORY_BODYPART_GLOVE_TAG =        "bodypart_glove";
        public readonly static string MEMORY_BODYPART_SHOE_TAG =         "bodypart_shoe";
        public readonly static string MEMORY_PLAYER_TAG =                "player";
        public readonly static string MEMORY_WIFI_SSID_TAG =             "wifi_ssid";
        public readonly static string MEMORY_WIFI_PASSWORD_TAG =         "wifi_password";
        public readonly static string MEMORY_WIFI_MULTICASTMESSAGE_TAG = "multicast_message";

        // SENSOR DATA TYPES
        public readonly static string SENSORTYPE_NONE_TAG =                   "none";
        public readonly static string SENSORTYPE_ORIENTATION_ABS_TAG =        "orientation_abs";
        public readonly static string SENSORTYPE_ACCELERATION_REL_TAG =       "acceleration_rel";
        public readonly static string SENSORTYPE_GLOVE_TAG =                  "glove";
        public readonly static string SENSORTYPE_SHOE_TAG =                   "shoe";
        public readonly static string SENSORTYPE_ANGULARVELOCITY_REL_TAG =    "angularvelocity_rel";

        // SENSOR
        public readonly static uint SENSOR_STATUS_NOT_ACCESSIBLE =  1;
        public readonly static uint SENSOR_STATUS_CALIBRATING =     2;
        public readonly static uint SENSOR_STATUS_WORKING =         3;

        // CONNECTION STATUS
        public readonly static uint CONNECTION_STATUS_NOT_CONNECTED = 0;
        public readonly static uint CONNECTION_STATUS_WAITING_ACK =   1;
        public readonly static uint CONNECTION_STATUS_CONNECTED =     2;

        // WIFI CONNECTIONS
        public readonly static uint WIFI_PORT =                         12345;
        public readonly static uint WIFI_MULTICAST_PORT =               12346;
        public readonly static string WIFI_SSID_DEFAULT =              "BodynodeHotspot";
        public readonly static string WIFI_PASSWORD_DEFAULT =          "bodynodes1";
        public readonly static string WIFI_MULTICASTGROUP_DEFAULT =    "239.192.1.99";
        public readonly static string WIFI_MULTICASTMESSAGE_DEFAULT =  "BN";

        // BLUETOOTH CONNECTIONS -> TODO

        // BLE CONNECTIONS
        public readonly static string BLE_NAME =                                    "Bodynode";
        public readonly static string BLE_SERVICE_UUID =                            "0000CCA0-0000-1000-8000-00805F9B34FB";
        public readonly static string BLE_CHARA_PLAYER_UUID =                       "0000CCA1-0000-1000-8000-00805F9B34FB";
        public readonly static string BLE_CHARA_BODYPART_UUID =                     "0000CCA2-0000-1000-8000-00805F9B34FB";
        public readonly static string BLE_CHARA_ORIENTATION_ABS_VALUE_UUID =        "0000CCA3-0000-1000-8000-00805F9B34FB";
        public readonly static string BLE_CHARA_ACCELERATION_REL_VALUE_UUID =       "0000CCA4-0000-1000-8000-00805F9B34FB";
        public readonly static string BLE_CHARA_GLOVE_VALUE_UUID =                  "0000CCA5-0000-1000-8000-00805F9B34FB";
        public readonly static string BLE_CHARA_SHOE_UUID =                         "0000CCA6-0000-1000-8000-00805F9B34FB";
        public readonly static string BLE_CHARA_ANGULARVELOCITY_REL_VALUE_UUID =    "0000CCA7-0000-1000-8000-00805F9B34FB";
    }
}