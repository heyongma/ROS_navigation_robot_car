#include <iostream>
#include <vector>
#include <string>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>

std::vector<pid_t> child_pids;

void sigint_handler(int sig)
{
std::cout << "\n收到Ctrl+C，关闭全部子进程" << std::endl;
for (auto pid : child_pids)
{
kill(pid, SIGINT);
waitpid(pid, nullptr, 0);
    }
std::cout << "全部进程关闭完毕" << std::endl;
_exit(0);
}

pid_t run_process(const std::vector<std::string>& cmd)
{
std::vector<const char*> argv;
for (auto &s : cmd) argv.push_back(s.c_str());
argv.push_back(nullptr);

pid_t pid = fork();
if (pid == 0)
{
execvp(argv[0], const_cast<char**>(argv.data()));
perror("execvp fail");
_exit(EXIT_FAILURE);
    }
return pid;
}

int main(int argc, char** argv)
{
signal(SIGINT, sigint_handler);
std::cout << "==== 一键启动整套仿真 ====" << std::endl;

    // 1. gazebo小车仿真
child_pids.push_back(run_process({"roslaunch", "lianxi_gazebo", "car_to.launch"}));
sleep(6);

    // 2. map_server01.launch
child_pids.push_back(run_process({"roslaunch", "lianxi_slam", "map_server01.launch"}));
sleep(2);

    // 3. cmal.launch
child_pids.push_back(run_process({"roslaunch", "lianxi_slam", "cmal.launch"}));
sleep(2);

    // 4. rviz
child_pids.push_back(run_process({"rosrun", "rviz", "rviz"}));
sleep(2);

    // 5. obs_auto 障碍物
child_pids.push_back(run_process({"rosrun", "lianxi_slam", "obs_auto"}));
sleep(2);

    // 6. 避让判定
child_pids.push_back(run_process({"rosrun", "lianxi_slam", "obs2_stop_and_decide_final"}));
sleep(2);

    // ======新增：path_show 路径可视化节点======
child_pids.push_back(run_process({"rosrun", "lianxi_slam", "path_show"}));
sleep(2);

std::cout << "✅ Gazebo、map_server、amcl、RVIZ、obs_auto、obs2_stop_and_decide_final、path_show 全部启动完成！" << std::endl;
std::cout << "👉 RViz Path插件填写话题：/show_path" << std::endl;

while (true)
{
sleep(1);
    }
return 0;
}
