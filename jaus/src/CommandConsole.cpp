/*
 * A command console class.
 * Copyright (c) 2008 Christopher D. Granz
 * All rights reserved.
 */

#include "CommandConsole.h"


/*
 * Constructor for internal CommandData class.
 */
CommandConsole::CommandData::CommandData( const char *CmdStr,
  void (*CmdFunc)( FILE *Console, char *Args[] ) )
{
    this->CommandStr  = new char[strlen(CmdStr) + 1];
    strcpy( this->CommandStr, CmdStr );
    this->CommandFunc = CmdFunc;
}


/*
 * Destructor for internal CommandData class.
 */
CommandConsole::CommandData::~CommandData( )
{
    delete this->CommandStr;
}


/*
 * Constructor
 */
CommandConsole::CommandConsole( const char *PromptStr,
  void (*CmdNotFoundFunc)( FILE *Console, char *Args[] ) )
{
    this->PromptStr = new char[strlen(PromptStr) + 1];
    strcpy( this->PromptStr, PromptStr );
    this->CommandNotFoundFunc = CmdNotFoundFunc;

    this->ThreadAlive   = 0;
    this->Console       = NULL;
    this->CommandList   = NULL;

    pthread_mutex_init( &this->Mutex, NULL );
}


/*
 * Destructor
 */
CommandConsole::~CommandConsole( )
{
    StopThread( );
    RemoveAllCommands( );
    delete PromptStr;
    pthread_mutex_destroy( &this->Mutex );
}


/*
 * Adds a command with the given name (CmdStr) to this CommandConsole object.
 */
int CommandConsole::AddCommand( const char *CmdStr,
                      void (*CmdFunc)( FILE *Console, char *Args[] ) )
{
    CommandData *p;

    p = new CommandData( CmdStr, CmdFunc );

    // lock while changing things the thread may be using
    pthread_mutex_lock( &Mutex );

    // link it in
    p->pNext          = this->CommandList;
    this->CommandList = p;

    pthread_mutex_unlock( &Mutex );

    return 0;
}


/*
 * Removes a command with the given name from this CommandConsole object.
 */
int CommandConsole::RemoveCommand( const char *CmdStr )
{
    CommandData *p, *pPrev = NULL;

    // lock while changing things the thread may be using
    pthread_mutex_lock( &Mutex );

    for ( p = CommandList; p; p = p->pNext )
    {
        if ( !strcmp( p->CommandStr, CmdStr ) )
            break;

        pPrev = p;
    }

    if ( !p )
    {
        pthread_mutex_unlock( &Mutex );
        return -1;
    }

    if ( pPrev )
        pPrev->pNext = p->pNext;
    else // if no previous, it must be first
        CommandList  = p->pNext;

    pthread_mutex_unlock( &Mutex );

    delete p;
    return 0;
}


/*
 * Removes all of the commands in this CommandConsole object.
 */
void CommandConsole::RemoveAllCommands( void )
{
    CommandData *p, *pNext;

    // lock while changing things the thread may be using
    pthread_mutex_lock( &Mutex );

    for ( p = this->CommandList; p; p = pNext )
    {
        pNext = p->pNext;
        delete p;
    }

    this->CommandList = NULL;

    pthread_mutex_unlock( &Mutex );
}


/*
 * Sets the displayed prompt string for this CommandConsole object.
 */
void CommandConsole::SetPrompt( const char *PromptStr )
{
    // lock while changing things the thread may be using
    pthread_mutex_lock( &Mutex );

    delete this->PromptStr;
    this->PromptStr = new char[strlen(PromptStr) + 1];
    strcpy( this->PromptStr, PromptStr );

    pthread_mutex_unlock( &Mutex );

    return;
}


/*
 * Returns a pointer to this objects display prompt string.
 */
char *CommandConsole::GetPrompt( void )
{
    return PromptStr;
}


/*
 * Starts the processing thread for this CommandConsole object.
 */
int CommandConsole::StartThread( int ConsoleFD )
{
    int fd;

    StopThread(); // stop previous if still running

    pthread_mutex_lock( &Mutex ); // lock while starting thread

    if ( (fd = dup( ConsoleFD )) < 0 ) // create a local duplicate fd
    {
        pthread_mutex_unlock( &Mutex );
        return -1;
    }

    // use the C file i/o library for convenience
    if ( !(Console = fdopen( fd, "r+" )) )
    {
        close( fd );
        pthread_mutex_unlock( &Mutex );
        return -2;
    }

    // create a thread to handle receiving commands
    ThreadAlive = 1;

    if ( pthread_create( &ThreadID, NULL, CommandConsole::RunThread,
      this ) != 0 )
    {
        fclose( Console );
        Console = NULL;
        pthread_mutex_unlock( &Mutex );
        return -3;
    }

    pthread_mutex_unlock( &Mutex );
    return 0;
}


/*
 * Will cause the current processing thread to terminate by closing the console
 * file descriptor.
 */
void CommandConsole::StopThread( void )
{
    pthread_mutex_lock( &Mutex );

    if ( Console )
    {
        // closing Console will cause thread to die
        fclose( Console );
        Console = NULL;
    }

    pthread_mutex_unlock( &Mutex );
}


/*
 * Returns non-zero if the command console thread is currently running.
 */
int CommandConsole::ThreadRunning( void )
{
    int i;

    pthread_mutex_lock( &Mutex );
    i = ThreadAlive;
    pthread_mutex_unlock( &Mutex );

    return i;
}


/*
 * Thread entry point after StartThread is called.
 * This thread will endless read input from the given file descriptor and
 * dispatch command functions if they are found.
 */
void *CommandConsole::RunThread( void *arg )
{
    CommandConsole *obj = (CommandConsole *) arg;
    CommandData *Cmd;
    char buf[512];
    char *p = buf;
    char *pSave;
    char *Args[COMMAND_CONSOLE_MAX_ARGS];
    char c;
    unsigned int i;

    //fprintf( obj->Console, "%s", obj->PromptStr );

    // loop until the console file descriptor is closed
    while( (c = getc( obj->Console )) != EOF )
    {
        if ( c == '\n' || (unsigned int) (p - buf) >= (sizeof( buf ) - 1) )
        {
            *p = '\0';
            p = buf;

            // parse the command
            Args[0] = strtok_r( p, "\t\r ", &pSave );

            if ( Args[0] == NULL ) // empty line
            {
                fprintf( obj->Console, "%s", obj->PromptStr );
                p = buf;
                continue;
            }

            // parse the arguments
            for ( i = 1; i < (sizeof( Args ) / sizeof( Args[1] )); i++ )
            {
                Args[i] = strtok_r( NULL, "\t\r ", &pSave );

                if ( Args[i] == NULL )
                    break;
            }

            // lookup the command
            pthread_mutex_lock( &obj->Mutex ); // lock the command list

            for ( Cmd = obj->CommandList; Cmd; Cmd = Cmd->pNext )
            {
                if ( !strcmp( Args[0], Cmd->CommandStr ) )
                    break;
            }

            // if not found call not found function of print message
            if ( !Cmd )
            {
                if ( obj->CommandNotFoundFunc )
                    obj->CommandNotFoundFunc( obj->Console, Args );
                else
                    fprintf( obj->Console, "Command not found.\n\n" );
            }
            else
                Cmd->CommandFunc( obj->Console, Args );

            pthread_mutex_unlock( &obj->Mutex ); // unlock the command list

            fprintf( obj->Console, "%s", obj->PromptStr );
            p = buf;
        }
        else
        {
            *p = c;
            p++;
        }
    }

    // lock while changing thread alive indicator
    pthread_mutex_lock( &obj->Mutex );
    obj->ThreadAlive = 0;
    pthread_mutex_unlock( &obj->Mutex );

    return NULL; // nothing to return
}


/*
 * End of CommandConsole.cpp
 */
