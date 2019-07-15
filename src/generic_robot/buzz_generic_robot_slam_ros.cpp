#include <buzz/buzzasm.h>
#include "utils/buzz_utils.h"
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <chrono>
#include <thread>

static int done = 0;

/*
 * Print usage information
 */
void usage(const char* path, int status) {
   fprintf(stderr, "Usage:\n");
   fprintf(stderr, "\t%s <stream> <msg_size> <file.bo> <file.bdb>\n\n", path);
   fprintf(stderr, "== Options ==\n\n");
   fprintf(stderr, "  stream        The stream type: tcp or bt\n");
   fprintf(stderr, "  msg_size      The message size in bytes\n");
   fprintf(stderr, "  file.bo       The Buzz bytecode file\n");
   fprintf(stderr, "  file.bdbg     The Buzz debug file\n\n");
   fprintf(stderr, "  robot_id     The ID Buzz assigned to the Buzz VM (should start from 0)\n\n");
   fprintf(stderr, "  port     The port number to use\n\n");
   exit(status);
}

static void ctrlc_handler(int sig) {
   done = 1;
}

int main(int argc, char** argv) {
   /* Parse command line */
   if(argc != 7) usage(argv[0], 0);
   /* The stream type */
   char* stream = argv[1];
   if(strcmp(stream, "tcp") != 0 &&
      strcmp(stream, "bt") != 0) {
      fprintf(stderr, "%s: unknown stream type '%s'\n", argv[0], stream);
      usage(argv[0], 0);
   }
   /* The message size */
   char* endptr;
   int msg_sz = strtol(argv[2], &endptr, 10);
   if(endptr == argv[2] || *endptr != '\0') {
      fprintf(stderr, "%s: can't parse '%s' into a number\n", argv[0], argv[2]);
      return 0;
   }
   if(msg_sz <= 0) {
      fprintf(stderr, "%s: invalid value %d for message size\n", argv[0], msg_sz);
      return 0;
   }
   /* The bytecode filename */
   char* bcfname = argv[3];
   /* The debugging information file name */
   char* dbgfname = argv[4];
   /* The robot id */
   int robot_id = strtol(argv[5], &endptr, 10);
   if(endptr == argv[5] || *endptr != '\0') {
      fprintf(stderr, "%s: can't parse '%s' into a number\n", argv[0], argv[5]);
      return 0;
   }
   /* The port to use */
   char* port = argv[6];

   /* Wait for connection */
   if(!buzz_listen(stream, msg_sz, port)) return 1;
   /* Set CTRL-C handler */
   signal(SIGTERM, ctrlc_handler);
   signal(SIGINT, ctrlc_handler);
   /* Initialize the robot */
   /* Set the Buzz bytecode */
   if(buzz_script_set<buzz_slam::BuzzSLAMRos>(robot_id, bcfname, dbgfname)) {
      /* Main loop */
      int step = 0;
      while(!done && !buzz_script_done()){
         std::cout << "Buzz VM Step " << step << std::endl;
         buzz_script_step();
         //std::this_thread::sleep_for(std::chrono::milliseconds(100));
         step++;
      }
      /* Cleanup */
      buzz_script_destroy();
   }
   
   /* Stop the robot */
   buzz_slam::BuzzSLAMSingleton::GetInstance().Destroy();
   /* All done */
   return 0;
}
