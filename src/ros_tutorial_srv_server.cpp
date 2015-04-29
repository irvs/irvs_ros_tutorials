#include "ros/ros.h"                            // ROSメインヘッダーファイル
#include "irvs_ros_tutorials/srvTutorial.h"  // srvTutorial サービスファイルのヘッダー（ビルド後に自動的に生成される）

#define PLUS           1    // 加算
#define MINUS          2    // 減算
#define MULTIPLICATION 3    // 乗算
#define DIVISION       4    // 割算

int g_operator = PLUS;

// サービスの要求があった場合は、以下の処理を実行する
// サービス要求は、 res 、サービスの応答は、 reqに設定した
bool calculation(irvs_ros_tutorials::srvTutorial::Request  &req,
                 irvs_ros_tutorials::srvTutorial::Response &res)
{
  // サービスリクエストを受けたaとbの値をパラメータ値に基づいて演算子を異にする。
  // 計算した後、サービスの応答値に保存する
  switch(g_operator){
    case PLUS:
         res.result = req.a + req.b; break;
    case MINUS:
         res.result = req.a - req.b; break;
    case MULTIPLICATION:
         res.result = req.a * req.b; break;  
    case DIVISION:
         if(req.b == 0){
           res.result = 0; break;
         }  
         else{
           res.result = req.a / req.b; break;  
         }
    default:
         res.result = req.a + req.b; break;
  }

  // サービスの要求に使用されたa 、bの値の表示およびサービスの応答に対応する
  ROS_INFO("request: x=%f, y=%f", req.a, req.b);
  ROS_INFO("sending back response: [%f]", res.result);

  return true;
}

int main(int argc, char **argv)                     // ノードのメイン関数
{
  ros::init(argc, argv, "ros_tutorial_srv_server"); // ノード名の初期化

  ros::NodeHandle nh;                     // ROSシステムとの通信のためのノードのハンドルを宣言
  ros::NodeHandle nh_priv("~");           // ROSシステムとの通信のためのノードのハンドルを宣言(Private)
                                          // http://wiki.ros.org/Names
  nh_priv.setParam("calculation_method", PLUS);   // パラメータの初期設定, http://wiki.ros.org/rosparam

  // サービスサーバを宣言し、 oroca_ros_tutorialsパッケージのsrvTutorialサービスファイルを利用した
  // サービスサーバros_tutorial_service_serverを作成する。サービス名は「 ros_tutorial_srv 」であり、
  // サービスの要求がある場合には、 calculationという名前の関数を実行する設定
  ros::ServiceServer ros_tutorial_service_server = nh.advertiseService("ros_tutorial_srv", calculation);

  ROS_INFO("ready srv server!");

  ros::Rate r(10); // 10 hz

  while (ros::ok())
  {
    nh_priv.getParam("calculation_method", g_operator);  // 演算子をパラメータから取得した値に変更する
    ros::spinOnce();  // コールバック関数の処理ルーチン
    r.sleep();        // ルーチンを繰り返すためのsleep処理
  }

  return 0;
}
