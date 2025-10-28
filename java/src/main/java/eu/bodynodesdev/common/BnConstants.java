/*
 * MIT License
 *
 * Copyright (c) 2015-2025 Manuel Bottini
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS"; WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package eu.bodynodesdev.common;

/**
 * Created by m.bottini on 04/12/2015.
 */
public class BnConstants {

    // Prevent instantiation
     private BnConstants() {
        throw new UnsupportedOperationException("Class cannot be instantiated");
    }

    // PLAYER NAMES CASES
    public static final String PLAYER_ALL_TAG =     "all";
    public static final String PLAYER_NONE_TAG =    "none";

    // BODYPARTS
    public static final String BODYPART_HEAD_TAG =            "head";
    public static final String BODYPART_HAND_LEFT_TAG =       "hand_left";
    public static final String BODYPART_LOWERARM_LEFT_TAG =   "lowerarm_left";
    public static final String BODYPART_UPPERARM_LEFT_TAG =   "upperarm_left";
    public static final String BODYPART_BODY_TAG =            "body";
    public static final String BODYPART_LOWERARM_RIGHT_TAG =  "lowerarm_right";
    public static final String BODYPART_UPPERARM_RIGHT_TAG =  "upperarm_right";
    public static final String BODYPART_HAND_RIGHT_TAG =      "hand_right";
    public static final String BODYPART_LOWERLEG_LEFT_TAG =   "lowerleg_left";
    public static final String BODYPART_UPPERLEG_LEFT_TAG =   "upperleg_left";
    public static final String BODYPART_FOOT_LEFT_TAG =       "foot_left";
    public static final String BODYPART_LOWERLEG_RIGHT_TAG =  "lowerleg_right";
    public static final String BODYPART_UPPERLEG_RIGHT_TAG =  "upperleg_right";
    public static final String BODYPART_FOOT_RIGHT_TAG =      "foot_right";
    public static final String BODYPART_UPPERBODY_TAG =       "upperbody";
    public static final String BODYPART_LOWERBODY_TAG =       "lowerbody";
    public static final String BODYPART_KATANA_TAG =          "katana";
    public static final String BODYPART_UNTAGGED_TAG =        "untagged";
    public static final String BODYPART_ALL_TAG =             "all";

    // ACTIONS
    public static final String ACTION_TYPE_TAG =              "type";
    public static final String ACTION_PLAYER_TAG =            "player";
    public static final String ACTION_BODYPART_TAG =          "bodypart";

    public static final String ACTION_TYPE_NONE_TAG =         "none";
    public static final String ACTION_TYPE_HAPTIC_TAG =       "haptic";
    public static final String ACTION_TYPE_SETPLAYER_TAG =    "set_player";
    public static final String ACTION_TYPE_SETBODYPART_TAG =  "set_bodypart";
    public static final String ACTION_TYPE_ENABLESENSOR_TAG = "enable_sensor";
    public static final String ACTION_TYPE_SETWIFI_TAG =      "set_wifi";

    public static final String ACTION_HAPTIC_DURATION_MS_TAG =          "duration_ms";
    public static final String ACTION_HAPTIC_STRENGTH_TAG =             "strength";
    public static final String ACTION_SETPLAYER_NEWPLAYER_TAG =         "new_player";
    public static final String ACTION_SETBODYPART_NEWBODYPART_TAG =     "new_bodypart";
    public static final String ACTION_ENABLESENSOR_SENSORTYPE_TAG =     "sensortype";
    public static final String ACTION_ENABLESENSOR_ENABLE_TAG =         "enable";
    public static final String ACTION_SETWIFI_SSID_TAG =                "ssid";
    public static final String ACTION_SETWIFI_PASSWORD_TAG =            "password";
    public static final String ACTION_SETWIFI_MULTICASTMESSAGE_TAG =    "multicast_message";

    // MESSAGE
    public static final String MESSAGE_PLAYER_TAG =        "player";
    public static final String MESSAGE_BODYPART_TAG =      "bodypart";
    public static final String MESSAGE_SENSORTYPE_TAG =    "sensortype";
    public static final String MESSAGE_VALUE_TAG =         "value";

    // GLOVE
    public static final int GLOVE_ANGLE_MIGNOLO_INDEX =  0;
    public static final int GLOVE_ANGLE_ANULARE_INDEX =  1;
    public static final int GLOVE_ANGLE_MEDIO_INDEX =    2;
    public static final int GLOVE_ANGLE_INDICE_INDEX =   3;
    public static final int GLOVE_ANGLE_POLLICE_INDEX =  4;
    public static final int GLOVE_TOUCH_MIGNOLO_INDEX =  5;
    public static final int GLOVE_TOUCH_ANULARE_INDEX =  6;
    public static final int GLOVE_TOUCH_MEDIO_INDEX =    7;
    public static final int GLOVE_TOUCH_INDICE_INDEX =   8;

    // MEMORY
    public static final String MEMORY_BODYPART_TAG =              "bodypart";
    public static final String MEMORY_BODYPART_GLOVE_TAG =        "bodypart_glove";
    public static final String MEMORY_BODYPART_SHOE_TAG =         "bodypart_shoe";
    public static final String MEMORY_PLAYER_TAG =                "player";
    public static final String MEMORY_WIFI_SSID_TAG =             "wifi_ssid";
    public static final String MEMORY_WIFI_PASSWORD_TAG =         "wifi_password";
    public static final String MEMORY_WIFI_MULTICASTMESSAGE_TAG = "multicast_message";

    // SENSOR DATA TYPES
    public static final String SENSORTYPE_NONE_TAG =                   "none";
    public static final String SENSORTYPE_ORIENTATION_ABS_TAG =        "orientation_abs";
    public static final String SENSORTYPE_ACCELERATION_REL_TAG =       "acceleration_rel";
    public static final String SENSORTYPE_GLOVE_TAG =                  "glove";
    public static final String SENSORTYPE_SHOE_TAG =                   "shoe";
    public static final String SENSORTYPE_ANGULARVELOCITY_REL_TAG =    "angularvelocity_rel";

    // SENSOR
    public static final int SENSOR_STATUS_NOT_ACCESSIBLE =  1;
    public static final int SENSOR_STATUS_CALIBRATING =     2;
    public static final int SENSOR_STATUS_WORKING =         3;

    // CONNECTION STATUS
    public static final int CONNECTION_STATUS_NOT_CONNECTED = 0;
    public static final int CONNECTION_STATUS_WAITING_ACK =   1;
    public static final int CONNECTION_STATUS_CONNECTED =     2;

    // WIFI CONNECTIONS
    public static final int WIFI_PORT =                         12345;
    public static final int WIFI_MULTICAST_PORT =               12346;
    public static final String WIFI_SSID_DEFAULT =              "BodynodeHotspot";
    public static final String WIFI_PASSWORD_DEFAULT =          "bodynodes1";
    public static final String WIFI_MULTICASTGROUP_DEFAULT =    "239.192.1.99";
    public static final String WIFI_MULTICASTMESSAGE_DEFAULT =  "BN";

    // BLUETOOTH CONNECTIONS -> TODO

    // BLE CONNECTIONS
    public static final String BLE_NAME =                                    "Bodynode";
    public static final String BLE_SERVICE_UUID =                            "0000CCA0-0000-1000-8000-00805F9B34FB";
    public static final String BLE_CHARA_PLAYER_UUID =                       "0000CCA1-0000-1000-8000-00805F9B34FB";
    public static final String BLE_CHARA_BODYPART_UUID =                     "0000CCA2-0000-1000-8000-00805F9B34FB";
    public static final String BLE_CHARA_ORIENTATION_ABS_VALUE_UUID =        "0000CCA3-0000-1000-8000-00805F9B34FB";
    public static final String BLE_CHARA_ACCELERATION_REL_VALUE_UUID =       "0000CCA4-0000-1000-8000-00805F9B34FB";
    public static final String BLE_CHARA_GLOVE_VALUE_UUID =                  "0000CCA5-0000-1000-8000-00805F9B34FB";
    public static final String BLE_CHARA_SHOE_UUID =                         "0000CCA6-0000-1000-8000-00805F9B34FB";
    public static final String BLE_CHARA_ANGULARVELOCITY_REL_VALUE_UUID =    "0000CCA7-0000-1000-8000-00805F9B34FB";
}
