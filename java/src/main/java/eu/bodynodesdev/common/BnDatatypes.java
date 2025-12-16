/**
 * MIT License
 *
 * <p>Copyright (c) 2024-2025 Manuel Bottini
 *
 * <p>Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * <p>The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * <p>THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
package eu.bodynodesdev.common;

public class BnDatatypes {
  public static final int MAX_RECEIVED_BYTES_LENGTH = 150;

  /*
      public class BnSensorType
      {
          private String value;

          public BnSensorType() {
              this.value = BnConstants.SENSORTYPE_NONE_TAG;
          }

          public String getValue() {
              return value;
          }

          public void setFromString(string type)
          {
              // SENSORTYPE
              if (BnConstants.SENSORTYPE_NONE_TAG.equals(type))
              {
                  value = BnConstants.SENSORTYPE_NONE_TAG;
              }
              else if (BnConstants.SENSORTYPE_ORIENTATION_ABS_TAG.equals(type))
              {
                  value = BnConstants.SENSORTYPE_ORIENTATION_ABS_TAG;
              }
              else if (BnConstants.SENSORTYPE_ACCELERATION_REL_TAG.equals(type))
              {
                  value = BnConstants.SENSORTYPE_ACCELERATION_REL_TAG;
              }
              else if (BnConstants.SENSORTYPE_GLOVE_TAG.equals(type))
              {
                  value = BnConstants.SENSORTYPE_GLOVE_TAG;
              }
              else if (BnConstants.SENSORTYPE_SHOE_TAG.equals(type))
              {
                  value = BnConstants.SENSORTYPE_SHOE_TAG;
              }
          }
      }

      public class BnBodypart
      {
          private String value;

          public BnBodypart() {
              this.value = BnConstants.BODYPART_NONE_TAG;
          }

          public String getValue() {
              return value;
          }

          void setFromString(string bodypart)
          {
              if (BnConstants.BODYPART_NONE_TAG.equals(bodypart))
              {
                  value = BnConstants.BODYPART_NONE_TAG;
              }
              else if (BnConstants.BODYPART_HEAD_TAG.equals(bodypart))
              {
                  value = BnConstants.BODYPART_HEAD_TAG;
              }
              else if (BnConstants.BODYPART_HAND_LEFT_TAG.equals(bodypart))
              {
                  value = BnConstants.BODYPART_HAND_LEFT_TAG;
              }
              else if (BnConstants.BODYPART_LOWERARM_LEFT_TAG.equals(bodypart))
              {
                  value = BnConstants.BODYPART_LOWERARM_LEFT_TAG;
              }
              else if (BnConstants.BODYPART_UPPERARM_LEFT_TAG.equals(bodypart))
              {
                  value = BnConstants.BODYPART_UPPERARM_LEFT_TAG;
              }
              else if (BnConstants.BODYPART_BODY_TAG.equals(bodypart))
              {
                  value = BnConstants.BODYPART_BODY_TAG;
              }
              else if (BnConstants.BODYPART_LOWERARM_RIGHT_TAG.equals(bodypart))
              {
                  value = BnConstants.BODYPART_LOWERARM_RIGHT_TAG;
              }
              else if (BnConstants.BODYPART_UPPERARM_RIGHT_TAG.equals(bodypart))
              {
                  value = BnConstants.BODYPART_UPPERARM_RIGHT_TAG;
              }
              else if (BnConstants.BODYPART_HAND_RIGHT_TAG.equals(bodypart))
              {
                  value = BnConstants.BODYPART_HAND_RIGHT_TAG;
              }
              else if (BnConstants.BODYPART_LOWERLEG_LEFT_TAG.equals(bodypart))
              {
                  value = BnConstants.BODYPART_LOWERLEG_LEFT_TAG;
              }
              else if (BnConstants.BODYPART_UPPERLEG_LEFT_TAG.equals(bodypart))
              {
                  value = BnConstants.BODYPART_UPPERLEG_LEFT_TAG;
              }
              else if (BnConstants.BODYPART_FOOT_LEFT_TAG.equals(bodypart))
              {
                  value = BnConstants.BODYPART_FOOT_LEFT_TAG;
              }
              else if (BnConstants.BODYPART_LOWERLEG_RIGHT_TAG.equals(bodypart))
              {
                  value = BnConstants.BODYPART_LOWERLEG_RIGHT_TAG;
              }
              else if (BnConstants.BODYPART_UPPERLEG_RIGHT_TAG.equals(bodypart))
              {
                  value = BnConstants.BODYPART_UPPERLEG_RIGHT_TAG;
              }
              else if (BnConstants.BODYPART_FOOT_RIGHT_TAG.equals(bodypart))
              {
                  value = BnConstants.BODYPART_FOOT_RIGHT_TAG;
              }
              else if (BnConstants.BODYPART_UPPERBODY_TAG.equals(bodypart))
              {
                  value = BnConstants.BODYPART_UPPERBODY_TAG;
              }
              else if (BnConstants.BODYPART_LOWERBODY_TAG.equals(bodypart))
              {
                  value = BnConstants.BODYPART_LOWERBODY_TAG;
              }
              else if (BnConstants.BODYPART_KATANA_TAG.equals(bodypart))
              {
                  value = BnConstants.BODYPART_KATANA_TAG;
              }
              else if (BnConstants.BODYPART_UNTAGGED_TAG.equals(bodypart))
              {
                  value = BnConstants.BODYPART_UNTAGGED_TAG;
              }
              else if (BnConstants.BODYPART_ALL_TAG.equals(bodypart))
              {
                  value = BnConstants.BODYPART_ALL_TAG;
              }
          }
      }

      public class BnPlayer
      {
          private String value;

          public BnPlayer() {
              this.value = BnConstants.PLAYER_NONE_TAG;
          }

          public String setValue(String newname) {
              this.value = newname;
          }

          public String getValue() {
              return value;
          }
      }


      public class BnIPConnectionData
      {
          public void setDisconnected()
          {
              conn_status = BnConstants.CONNECTION_STATUS_NOT_CONNECTED;
          }
          public void setWaitingACK()
          {
              conn_status = BnConstants.CONNECTION_STATUS_WAITING_ACK;
          }
          public void setConnected()
          {
              conn_status = BnConstants.CONNECTION_STATUS_CONNECTED;
          }
          public bool isWaitingACK()
          {
              return conn_status == BnConstants.CONNECTION_STATUS_WAITING_ACK;
          }

          public bool isDisconnected()
          {
              return conn_status == BnConstants.CONNECTION_STATUS_NOT_CONNECTED;
          }
          public bool isConnected()
          {
              return conn_status == BnConstants.CONNECTION_STATUS_CONNECTED;
          }
          public void cleanBytes()
          {
              num_received_bytes = 0;
          }

          private int conn_status = BnConstants.CONNECTION_STATUS_NOT_CONNECTED;

          // TOFIX
          // IPEndPoint ip_address;

          bool has_ip_address;

          byte[] received_bytes = new byte[MAX_RECEIVED_BYTES_LENGTH];
          int num_received_bytes = 0;

          long last_sent_time = 0;
          long last_rec_time = 0;
      }

      public class BnSensorData
      {
          public void setValues(float[] values, BnSensorType sensortype)
          {
              sd_num_values = 0;
              if (sensortype.value == BnConstants.SENSORTYPE_ORIENTATION_ABS_TAG)
              {
                  sd_sensortype.value = sensortype.value;
                  sd_num_values = 4;
              }
              else if (sensortype.value == BnConstants.SENSORTYPE_ACCELERATION_REL_TAG)
              {
                  sd_sensortype.value = sensortype.value;
                  sd_num_values = 3;
              }
              sd_values_float = new float[sd_num_values];
              for (uint index = 0; index < sd_num_values; ++index)
              {
                  sd_values_float[index] = values[index];
              }
          }

          public void setValues(int[] values, BnSensorType sensortype)
          {
              sd_num_values = 0;
              if (sensortype.value == BnConstants.SENSORTYPE_GLOVE_TAG)
              {
                  sd_num_values = 9;
                  sd_sensortype = sensortype;
              }
              else if (sensortype.value == BnConstants.SENSORTYPE_SHOE_TAG)
              {
                  sd_num_values = 1;
                  sd_sensortype.value = sensortype.value;
              }
              sd_values_int = new int[sd_num_values];
              for (int index = 0; index < sd_num_values; ++index)
              {
                  sd_values_int[index] = values[index];
              }
          }

          public float[] getValuesFloat()
          {
              return sd_values_float;
          }

          public int[] getValuesInt()
          {
              return sd_values_int;
          }

          public String getValuesStr()
          {
              return sd_values_str;
          }

          public BnSensorType getSensorType()
          {
              return sd_sensortype;
          }

          public bool isEmpty()
          {
              return sd_sensortype.value == BnConstants.SENSORTYPE_NONE_TAG;
          }

          private uint sd_num_values = 0;
          private BnSensorType sd_sensortype = new BnSensorType();
          private float[] sd_values_float = new float[4];
          private int[] sd_values_int = new int[9];
          private String sd_values_str = "";
      };

      public class BnMessage
      {
          public void create(BnPlayer player, BnBodypart bodypart, BnSensorData sensorData)
          {
              m_player = player;
              m_bodypart = bodypart;
              m_sensorData = sensorData;
          }

          public void print()
          {
              String debugStr = "";
              debugStr += "Message = { ";
              debugStr += " player = ";
              debugStr += m_player.value;
              debugStr += " , ";
              debugStr += " bodypart = ";
              debugStr += m_bodypart.value;
              debugStr += " , ";
              debugStr += " value = ";
              if (m_sensorData.getSensorType().value == BnConstants.SENSORTYPE_ORIENTATION_ABS_TAG)
              {
                  float[] values = m_sensorData.getValuesFloat();
                  debugStr += values[0];
                  debugStr += " ";
                  debugStr += values[1];
                  debugStr += " ";
                  debugStr += values[2];
                  debugStr += " ";
                  debugStr += values[3];
              }
              else if (m_sensorData.getSensorType().value == BnConstants.SENSORTYPE_ACCELERATION_REL_TAG)
              {
                  float[] values = m_sensorData.getValuesFloat();
                  debugStr += values[0];
                  debugStr += " ";
                  debugStr += values[1];
                  debugStr += " ";
                  debugStr += values[2];
              }
              else if (m_sensorData.getSensorType().value == BnConstants.SENSORTYPE_GLOVE_TAG)
              {
                  Int32[] values = m_sensorData.getValuesInt();
                  debugStr += values[0];
                  debugStr += " ";
                  debugStr += values[1];
                  debugStr += " ";
                  debugStr += values[2];
                  debugStr += " ";
                  debugStr += values[3];
                  debugStr += " ";
                  debugStr += values[4];
                  debugStr += " ";
                  debugStr += values[5];
                  debugStr += " ";
                  debugStr += values[6];
                  debugStr += " ";
                  debugStr += values[7];
                  debugStr += " ";
                  debugStr += values[8];
              }
              else if (m_sensorData.getSensorType().value == BnConstants.SENSORTYPE_SHOE_TAG)
              {
                  Int32[] values = m_sensorData.getValuesInt();
                  debugStr += values[0];
              }
              debugStr += " }";
              System.out.println(debugStr);
          }

          public bool parseString(String messageStr)
          {

              ObjectMapper mapper = new ObjectMapper();
              JsonNode messageJson;
              try
              {
                  messageJson = mapper.readTree(messageStr);
              }
              catch (Exception ex)
              {
                  System.out.println("JSON parsing error: " + ex.Message);
                  return false;
              }

              if ( messageJson.get(BnConstants.MESSAGE_PLAYER_TAG) == null ||
                      messageJson.get(BnConstants.MESSAGE_BODYPART_TAG) == null ||
                      messageJson.get(BnConstants.MESSAGE_BODYPART_TAG) == null ||
                      messageJson.get(BnConstants.MESSAGE_VALUE_TAG) == null ) {
                  return false;
              }

              m_player.value =  messageJson.get(BnConstants.MESSAGE_PLAYER_TAG).asText();
              m_bodypart.value = messageJson.get(BnConstants.MESSAGE_BODYPART_TAG).asText();
              BnSensorType sensortype = new BnSensorType{ value = messageJson.get(BnConstants.MESSAGE_SENSORTYPE_TAG).asText() };

              if( BnConstants.SENSORTYPE_GLOVE_TAG.equals( sensortype.value ) )
              {
                  JsonNode valueNode = messageJson.get(BnConstants.MESSAGE_VALUE_TAG); // expected to be an array
                  int values[] = new int[valueNode.size()];
                  int counter = 0;
                  for (JsonNode scoreNode : valueNode) {
                      values[counter] = scoreNode.asInt();
                      counter++;
                  }
                  m_sensorData.setValues(values, sensortype);
              }
              else
              {
                  JsonNode valueNode = messageJson.get(BnConstants.MESSAGE_VALUE_TAG); // expected to be an array
                  float values[] = new float[valueNode.size()];
                  int counter = 0;
                  for (JsonNode scoreNode : valueNode) {
                      values[counter] = scoreNode.asFloat();
                      counter++;
                  }
                  m_sensorData.setValues(values, sensortype);
              }
              return true;
          }

          public BnPlayer getPlayer()
          {
              return m_player;
          }

          public BnBodypart getBodypart()
          {
              return m_bodypart;
          }

          public BnSensorData getData()
          {
              return m_sensorData;
          }

          private BnPlayer m_player = new BnPlayer();
          private BnBodypart m_bodypart = new BnBodypart()
          private BnSensorData m_sensorData = new BnSensorData();
      };

      public class BnAction
      {
          public BnAction()
          {
  #if USE_NEWTONSOFT_JSON
              a_json = new JObject();
              a_json.Add(BnConstants.ACTION_TYPE_TAG, BnConstants.ACTION_TYPE_NONE_TAG);
              a_json.Add(BnConstants.ACTION_PLAYER_TAG, BnConstants.PLAYER_NONE_TAG);
              a_json.Add(BnConstants.ACTION_BODYPART_TAG, BnConstants.BODYPART_NONE_TAG);
  #else
              a_json = new JsonObject();
              a_json[BnConstants.ACTION_TYPE_TAG] = BnConstants.ACTION_TYPE_NONE_TAG;
              a_json[BnConstants.ACTION_PLAYER_TAG] = BnConstants.PLAYER_NONE_TAG;
              a_json[BnConstants.ACTION_BODYPART_TAG] = BnConstants.BODYPART_NONE_TAG;
  #endif
          }

          public void createHaptic(BnPlayer player, BnBodypart bodypart, ushort duration_ms, ushort strength)
          {
  #if USE_NEWTONSOFT_JSON
              a_json = new JObject();
              a_json.Add(BnConstants.ACTION_TYPE_TAG, BnConstants.ACTION_TYPE_HAPTIC_TAG);
              a_json.Add(BnConstants.ACTION_PLAYER_TAG, player.value);
              a_json.Add(BnConstants.ACTION_BODYPART_TAG, bodypart.value);
              a_json.Add(BnConstants.ACTION_HAPTIC_DURATION_MS_TAG, duration_ms);
              a_json.Add(BnConstants.ACTION_HAPTIC_STRENGTH_TAG, strength);
  #else
              a_json = new JsonObject();
              a_json[BnConstants.ACTION_TYPE_TAG] = BnConstants.ACTION_TYPE_HAPTIC_TAG;
              a_json[BnConstants.ACTION_PLAYER_TAG] = player.value;
              a_json[BnConstants.ACTION_BODYPART_TAG] = bodypart.value;
              a_json[BnConstants.ACTION_HAPTIC_DURATION_MS_TAG] = duration_ms;
              a_json[BnConstants.ACTION_HAPTIC_STRENGTH_TAG] = strength;
  #endif
          }

          public void createSetPlayer(BnPlayer player, BnBodypart bodypart, BnPlayer new_player)
          {
  #if USE_NEWTONSOFT_JSON
              a_json = new JObject();
              a_json.Add(BnConstants.ACTION_TYPE_TAG, BnConstants.ACTION_TYPE_SETPLAYER_TAG);
              a_json.Add(BnConstants.ACTION_PLAYER_TAG, player.value);
              a_json.Add(BnConstants.ACTION_BODYPART_TAG, bodypart.value);
              a_json.Add(BnConstants.ACTION_SETPLAYER_NEWPLAYER_TAG, new_player.value);
  #else
              a_json = new JsonObject();
              a_json[BnConstants.ACTION_TYPE_TAG] = BnConstants.ACTION_TYPE_SETPLAYER_TAG;
              a_json[BnConstants.ACTION_PLAYER_TAG] = player.value;
              a_json[BnConstants.ACTION_BODYPART_TAG] = bodypart.value;
              a_json[BnConstants.ACTION_SETPLAYER_NEWPLAYER_TAG] = new_player.value;
  #endif
          }

          public void createSetBodypart(BnPlayer player, BnBodypart bodypart, BnBodypart new_bodypart)
          {
  #if USE_NEWTONSOFT_JSON
              a_json = new JObject();
              a_json.Add(BnConstants.ACTION_TYPE_TAG, BnConstants.ACTION_TYPE_SETBODYPART_TAG);
              a_json.Add(BnConstants.ACTION_PLAYER_TAG, player.value);
              a_json.Add(BnConstants.ACTION_BODYPART_TAG, bodypart.value);
              a_json.Add(BnConstants.ACTION_SETBODYPART_NEWBODYPART_TAG, new_bodypart.value);
  #else
              a_json = new JsonObject();
              a_json[BnConstants.ACTION_TYPE_TAG] = BnConstants.ACTION_TYPE_SETBODYPART_TAG;
              a_json[BnConstants.ACTION_PLAYER_TAG] = player.value;
              a_json[BnConstants.ACTION_BODYPART_TAG] = bodypart.value;
              a_json[BnConstants.ACTION_SETBODYPART_NEWBODYPART_TAG] = new_bodypart.value;
  #endif
          }

          public void createEnableSensor(BnPlayer player, BnBodypart bodypart, BnSensorType sensortype, bool toEnable)
          {
  #if USE_NEWTONSOFT_JSON
              a_json = new JObject();
              a_json.Add(BnConstants.ACTION_TYPE_TAG, BnConstants.ACTION_TYPE_ENABLESENSOR_TAG);
              a_json.Add(BnConstants.ACTION_PLAYER_TAG, player.value);
              a_json.Add(BnConstants.ACTION_BODYPART_TAG, bodypart.value);
              a_json.Add(BnConstants.ACTION_ENABLESENSOR_SENSORTYPE_TAG, bodypart.value);
              a_json.Add(BnConstants.ACTION_ENABLESENSOR_ENABLE_TAG, toEnable);
  #else
              a_json = new JsonObject();
              a_json[BnConstants.ACTION_TYPE_TAG] = BnConstants.ACTION_TYPE_ENABLESENSOR_TAG;
              a_json[BnConstants.ACTION_PLAYER_TAG] = player.value;
              a_json[BnConstants.ACTION_BODYPART_TAG] = bodypart.value;
              a_json[BnConstants.ACTION_ENABLESENSOR_SENSORTYPE_TAG] = bodypart.value;
              a_json[BnConstants.ACTION_ENABLESENSOR_ENABLE_TAG] = toEnable;
  #endif
          }

          public String getString()
          {
  #if USE_NEWTONSOFT_JSON
              return a_json.ToString();
  #else
              return a_json.ToJsonString();
  #endif
          }

          public bool isEmpty()
          {
              if (a_json == null || a_json[BnConstants.ACTION_PLAYER_TAG] == null )
              {
                  return true;
              }
              if ( BnConstants.PLAYER_NONE_TAG.equals(a_json[BnConstants.ACTION_PLAYER_TAG]!.ToString()))
              {
                  return true;
              }
              else
              {
                  return false;
              }
          }

          public BnSensorType getSensorType()
          {
              if (a_json == null || a_json[BnConstants.ACTION_TYPE_TAG] == null )
              {
                  return new BnSensorType { value = BnConstants.ACTION_TYPE_NONE_TAG };
              }

              return new BnSensorType { value = a_json[BnConstants.ACTION_TYPE_TAG]!.ToString() };
          }

          public BnBodypart getBodypart()
          {
              if (a_json == null || a_json[BnConstants.ACTION_BODYPART_TAG] == null )
              {
                  return new BnBodypart { value = BnConstants.BODYPART_NONE_TAG };
              }
              return new BnBodypart { value = a_json[BnConstants.ACTION_BODYPART_TAG]!.ToString() };
          }

          public BnPlayer getPlayer()
          {
              if (a_json == null || a_json[BnConstants.ACTION_PLAYER_TAG] == null )
              {
                  return new BnPlayer { value = BnConstants.PLAYER_NONE_TAG };
              }
              return new BnPlayer { value = a_json[BnConstants.ACTION_PLAYER_TAG]!.ToString() };
          }

          public bool parseString(String actionStr) {
  #if USE_NEWTONSOFT_JSON
              a_json = JObject.Parse(actionStr);
              if (a_json != null)
              {
                  return true;
              }
              else
              {
                  return false;
              }
  #else
              try
              {
                  a_json = JsonNode.Parse(actionStr)!.AsObject();
              }
              catch (JsonException ex)
              {
                  a_json = new JsonObject();
                  Console.WriteLine("JSON parsing error: " + ex.Message);
                  return false;
              }
              return true;
  #endif
          }

          public override string ToString()
          {
  #if USE_NEWTONSOFT_JSON
              string printString = a_json.ToString();
  #else
              string printString = a_json.ToJsonString();
  #endif
              return printString;
          }

          public void print()
          {
              Console.WriteLine(ToString());
          }

  #if USE_NEWTONSOFT_JSON
          private JObject a_json = null;
  #else
          private JsonObject a_json;
  #endif
      };
  */

}
