#include <csignal>
#include <cstdlib>
#include <iomanip>
#include "configuration.hpp"
#include "robot/humanoid.hpp"
#include "options/options.hpp"
#include "player/player.hpp"
#include "common.hpp"
#include "player/server/server.hpp"
using namespace std;
using namespace robot;

shared_ptr<player> maxwell;

void exit_handler(int sig)
{
    LOG << "\n--------------------------------------------------------\n"
        << "                         Good bye!                      \n"
        << "--------------------------------------------------------" << ENDL;

    if (sig == SIGINT)
    {
        maxwell->stop();
        sleep(1);
    }
}

void greeting()
{
    LOG << "\n---------------------------------------------------------------------\n"
        " .---. ----- .    .     .    .         .---.        .           .    \n"
        "|      |     |    |     |    |         |    \\       |           |    \n"
        "|      |     |    |     |    | .---. ` |    /       |          -|--  \n"
        " '---. ----- |    | ___ |    | |   | | |---'   .-.  |.-.   .-.  |    \n"
        "     | |     |    |     |    | |   | | |   \\  /   \\ |   \\ /   \\ |    \n"
        "     | |     |    |     |    | |   | | |    | \\   / |   / \\   / |    \n"
        "'---'  -----  '--'       '--'  '   ' ' '    '  '-'   '-'   '-'  '--  \n"
        "---------------------------------------------------------------------\n"
        "  Southeast University, Nanjing, China\n"
        "  Author: Liu Chuan.\n"
        "  All rights reserved.     \n"
        "---------------------------------------------------------------------\n"
        << left << setw(15) << "team-name: " << CONF->get_config_value<string>("team_name") << "\n"
        << left << setw(15) << "team-number: " << CONF->get_config_value<string>("team_number") << "\n"
        << left << setw(15) << "player-id: " << CONF->id() << "\n"
        <<  "--------------------------------------------------------" << ENDL;
}

int main(int argc, char *argv[])
{
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    cout.setf(ios::left);

    if (!OPTS->init(argc, argv))
    {
        LOG << "options init failed" << ENDL;
        return 1;
    }

    if (!CONF->init(OPTS->id()))
    {
        LOG << "config init failed" << ENDL;
        return 2;
    }

    greeting();

    ROBOT->init(CONF->robot_file(), CONF->action_file(), CONF->offset_file());

    maxwell = make_shared<player>();

    if (!maxwell->init())
    {
        LOG << "robot init failed" << ENDL;
        return 3;
    }

    while (maxwell->is_alive())
    {
        sleep(2);
    }

    return 0;
}
