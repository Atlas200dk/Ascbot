#include "stream_process.h"


int main(int argc, char *argv[]) {
  int ret = ascend::ascendstream::kStreamProcessOk;
  ascend::ascendstream::StreamProcess stream_process;

  // initialization
  ret = stream_process.StreamInit(argc, argv);
  if (ret != ascend::ascendstream::kStreamProcessOk) {
    return ret;
  }

  // begin to work
  ret = stream_process.StreamRun();
  if (ret != ascend::ascendstream::kStreamProcessOk) {
    return ret;
  }
  return ascend::ascendstream::kStreamProcessOk;
}

