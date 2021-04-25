#include "rclcpp/rclcpp.hpp"
#include "ros_assignments/srv/user_command.hpp"

#include <cstdlib>
#include <ncurses.h>

using UserCommand = ros_assignments::srv::UserCommand;
using namespace std::chrono_literals;


class UserInterface : public rclcpp::Node
{
public:
    UserInterface() :
        Node("user_interface")
    {
        timer_ = this->create_wall_timer(20ms, std::bind(&UserInterface::update, this));
        client_ = this->create_client<UserCommand>("user_command_service");

        init_screen();
    }

private:
    void init_screen()
    {
        /* Initialize curses and options.*/
        screen_ = initscr();
        cbreak();
        curs_set(false);
        noecho();
        nodelay(screen_, true);

        /* Welcome message. */
        mvaddstr(0, 0, "*** User interface ***");
        mvaddstr(2, 0, "Available commands: q - close, i - idle, r - random");
        mvaddstr(3, 0, "                    w - go forward, a - turn left, s - go backward, d - turn right");
        draw_status();
        draw_robot();
        refresh();
    }


    void draw_status()
    {
        mvaddstr(5, 0, "Status:          ");
        refresh();
        mvaddstr(5, 0, std::string("Status: " + status_).c_str());
    };


    void draw_robot()
    {

        if (robot_status_ == "up")
        {
            mvaddstr(7 , 4, "     ^    ");
            mvaddstr(8 , 4, " _   |   _");
        }
        else
        {
            mvaddstr(7 , 4, "          ");
            mvaddstr(8 , 4, " _       _");
        }

        if (robot_status_ == "right")
            mvaddstr(9 , 4, "/_/_____/_/");
        else if (robot_status_ == "left")
            mvaddstr(9 , 4, "\\_\\_____\\_\\");
        else
            mvaddstr(9 , 4, "|_|_____|_|");
        mvaddstr(10, 4, " |       |");
        mvaddstr(11, 4, " |       |");
        mvaddstr(12, 4, " |       |");
        mvaddstr(13, 4, " | robot |");
        mvaddstr(14, 4, " |       |");
        mvaddstr(15, 4, "|_|_____|_|");

        if (robot_status_ == "down")
        {
            mvaddstr(17 , 4, "     |    ");
            mvaddstr(18 , 4, "     V    ");
        }
        else
        {
            mvaddstr(17 , 4, "          ");
            mvaddstr(18 , 4, "          ");
        }
    };


    void close()
    {
        nocbreak();
        echo();
        endwin();
    };


    void send_command
    (
        const std::string& mode, const double& v_forward = 0.0, const double& w_yaw = 0.0
    )
    {
        auto request = std::make_shared<UserCommand::Request>();
        request->mode = mode;
        request->v_forward = v_forward;
        request->w_yaw = w_yaw;

        client_->async_send_request(request);
    };


    void update()
    {
        last_robot_status_ = robot_status_;

        char c = getch();
        switch (c)
        {
        case 'q':
            send_command("idle");
            close();
            exit(EXIT_SUCCESS);

            break;

        case 'r':
            status_ = "random";

            break;

        case 'i':
            status_ = "idle";
            robot_status_ = "idle";

            break;

        case 'w':
            status_ = "joystick";
            v_forward_ = 0.5;
            w_yaw_ = 0.0;
            robot_status_ = "up";

            break;

        case 'a':
            status_ = "joystick";
            v_forward_ = 0.0;
            w_yaw_ = 1.0;
            robot_status_ = "left";

            break;

        case 's':
            status_ = "joystick";
            v_forward_ = -0.5;
            w_yaw_ = 0.0;
            robot_status_ = "down";

            break;

        case 'd':
            status_ = "joystick";
            v_forward_ = 0.0;
            w_yaw_ = -1.0;
            robot_status_ = "right";

            break;
        }

        if (status_ == "joystick")
        {
            elapsed_ += period_ / 1000.0;

            if ((last_sent_ != "joystick") || (robot_status_ != last_robot_status_))
            {
                send_command("joystick", v_forward_, w_yaw_);
                last_sent_ = "joystick";
            }

            // if ((elapsed_ > 0.3) && (c != 'w') && (c != 'a') && (c != 's') && (c != 'd'))
            // {
            //     elapsed_ = 0.0;
            //     status_ = "idle";
            //     robot_status_ = "idle";
            // }
        }

        if (status_ == "idle")
        {
            if (last_sent_ != "idle")
            {
                send_command("idle");
                last_sent_ = "idle";
            }
        }
        else if (status_ == "random")
        {
            if (last_sent_ != "random")
            {
                send_command("random");
                last_sent_ = "random";
            }
        }

        draw_status();
        draw_robot();
        refresh();
    };

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<UserCommand>::SharedPtr client_;

    /* Window. */
    WINDOW* screen_;

    /* Status. */
    std::string status_ = "idle";
    std::string last_sent_ = "idle";
    int period_ = 20;
    double elapsed_ = 0.0;

    /* Robot data. */
    double v_forward_ = 0.0;
    double w_yaw_ = 0.0;
    std::string robot_status_ = "idle";
    std::string last_robot_status_ = "idle";
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UserInterface>());
    rclcpp::shutdown();

    nocbreak();
    echo();
    endwin();

    return EXIT_SUCCESS;
}
