#include "ros/ros.h"                        // ROSメインヘッダーファイル
#include "irvs_ros_tutorials/srvTutorial.h" // srvTutorial サービスファイルのヘッダー（ビルド後に自動的に生成される）
#include <cstdlib>

int main(int argc, char **argv)                     // ノードのメイン関数
{
  ros::init(argc, argv, "ros_tutorial_srv_client"); // ノード名の初期化
  if (argc != 3)                                    // 入力値のエラー処理
  {
    ROS_ERROR("cmd : rosrun ros_tutorial ros_tutorial_service_client arg0 arg1");
    ROS_ERROR("arg0: double number, arg1: double number");
    return 1;
  }

  ros::NodeHandle nh;   // ROSシステムとの通信のためのノードのハンドルを宣言

  // サービスクライアントを宣言し、 irvs_ros_tutorialsパッケージのsrvTutorialサービスファイルを利用した
  // サービスクライアントros_tutorial_service_clientを作成する。サービス名は「 ros_tutorial_srv 」である
  ros::ServiceClient ros_tutorial_service_client = nh.serviceClient<irvs_ros_tutorials::srvTutorial>("ros_tutorial_srv");

  // srvという名前でsrvTutorialサービス·ファイルを使用するサービス·ファイルを宣言する
  irvs_ros_tutorials::srvTutorial srv;

  // サービスの要求値にノードが実行されるときに、入力として使用されたパラメータを、それぞれのa 、bに格納する
  srv.request.a = atof(argv[1]);
  srv.request.b = atof(argv[2]);

  // サービスを要求し、要求が受け入れられた場合、応答値を表示する
  if (ros_tutorial_service_client.call(srv))
  {
    ROS_INFO("send srv, srv.Request.a and b: %f, %f", srv.request.a, srv.request.b);
    ROS_INFO("recieve srv, srv.Response.result: %f", srv.response.result);
  }
  else
  {
    ROS_ERROR("Failed to call service ros_tutorial_srv");
    return 1;
  }

  return 0;
}
