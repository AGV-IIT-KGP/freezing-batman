
/*
 * A command console class.
 * Copyright (c) 2008 Christopher D. Granz
 * All rights reserved.
 */

#ifndef __CommandConsoleH__
#define __CommandConsoleH__

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>

// maximum number of arguments for command functions
#define COMMAND_CONSOLE_MAX_ARGS	20

class CommandConsole
{
    private:
        class CommandData
        {
            public:
                char *CommandStr;
                void (*CommandFunc)( FILE *Console, char *Args[] );
                CommandData *pNext;

                CommandData( const char *CmdStr,
                  void (*CmdFunc)( FILE *Console, char *Args[] ) );
                ~CommandData( );
        };

        char *PromptStr;
        CommandData *CommandList;
        void (*CommandNotFoundFunc)( FILE *Console, char *Args[] );

        FILE *Console;
        pthread_t ThreadID;
        int ThreadAlive;

        pthread_mutex_t Mutex;

    public:
        CommandConsole( const char *PromptStr,
          void (*CmdNotFoundFunc)( FILE *Console, char *Args[] ) );
        ~CommandConsole( );

        int AddCommand( const char *CmdStr,
              void (*CmdFunc)( FILE *Console, char *Args[] ) );
        int RemoveCommand( const char *CmdStr );
        void RemoveAllCommands( void );
        void SetPrompt( const char *PromptStr );
        char *GetPrompt( void );

        int StartThread( const int ConsoleFD );
        void StopThread( void );
        int ThreadRunning( void );

    private:
        static void *RunThread( void *arg );
};


#endif // __CommandConsoleH__

/*
 * End of CommandConsole.h
 */

