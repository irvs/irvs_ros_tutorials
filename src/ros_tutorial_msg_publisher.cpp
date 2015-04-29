#include "ros/ros.h"                                    // ROSメインヘッダーファイル
#include "rostms_tutorial_basic/msgTutorial.h"          // msgTutorialメッセージファイルのヘッダー（ビルド後に自動的に生成される）

int main(int argc, char **argv)                         // ノードのメイン関数
{
  ros::init(argc, argv, "ros_tutorial_msg_publisher");  // ノード名の初期化
  ros::NodeHandle nh;                                   // ROSシステムとの通信のためのノードのハンドルを宣言

  // 発行者の宣言、 rostms_tutorial_basicパッケージのmsgTutorialメッセージファイルを利用した
  // 発行者ros_tutorial_pubを作成する。トピック名は " ros_tutorial_msg 」であり、
  // 発行キュー（ queue ）のサイズを100に設定するというものである
  // !!! http://wiki.ros.org/msg
  ros::Publisher ros_tutorial_pub = nh.advertise<rostms_tutorial_basic::msgTutorial>("ros_tutorial_msg", 100);

  // ループの周期を設定します。 "10"というのは、 10Hzを言うことで、0.1秒間隔で繰り返される
  // http://wiki.ros.org/roscpp/Overview/Time
  ros::Rate loop_rate(10); 

  int count = 0;    // メッセージに使用する変数の宣言

  while (ros::ok())
  {
    rostms_tutorial_basic::msgTutorial msg;      // msgTutorialメッセージファイル形式でmsgというメッセージを宣言
    msg.data = count;                   // countという変数を使用して、メッセージの値を定める

    ROS_INFO("send msg = %d", count);   // ROS_INFOというROS関数を使用して、 count変数を表示する

    ros_tutorial_pub.publish(msg);      // メッセージを発行する。約0.1秒間隔で発行される

    loop_rate.sleep();                  // 上で定められたループのサイクルに応じて、スリップに入る

    ++count;                            // count変数に1ずつ増加
  }

  return 0;
}
