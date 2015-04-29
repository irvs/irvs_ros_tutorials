#include "ros/ros.h"                                    // ROSメインヘッダーファイル
#include "rostms_tutorial_basic/msgTutorial.h"          // msgTutorialメッセージファイルのヘッダー（ビルド後に自動的に生成される）

// メッセージのコールバック関数として、下に設定したros_tutorial_sub者に該当するメッセージを
// 受信したときに動作する関数である
// 入力メッセージにはoroca_ros_tutorialパッケージのmsgTutorialメッセージを受信するようになっている
void msgCallback(const rostms_tutorial_basic::msgTutorial::ConstPtr& msg)
{
  ROS_INFO("recieve msg: %d", msg->data);   // 受信したメッセージを表示する関数
}

int main(int argc, char **argv)                         // ノードのメイン関数
{
  ros::init(argc, argv, "ros_tutorial_msg_subscriber"); // ノード名の初期化

  ros::NodeHandle nh;                                   // ROSシステムとの通信のためのノードのハンドルを宣言

  // 購読者を宣言し、 rostms_tutorial_basicパッケージのmsgTutorialメッセージファイルを利用した
  // 購読者ros_tutorial_subを作成する。トピック名は " ros_tutorial_msg 」であり、
  // サブスクライバのキュー（ queue ）のサイズを100に設定するというものである
  ros::Subscriber ros_tutorial_sub = nh.subscribe("ros_tutorial_msg", 100, msgCallback);

  // コールバック関数を呼び出すための関数として、メッセージが受信されることを待って、受信された場合、コールバック関数を実行する
  ros::spin();

  return 0;
}
