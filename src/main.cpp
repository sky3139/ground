
#include "ground_detection.h"

#include <iostream>
#include <stdio.h>
#include <sys/io.h>
#include <string>
#include <fstream>
#include <dirent.h>

#include <vector>
#include <chrono>
#include <cassert>
#include <functional>

#include "threadpool.h"

using namespace std;
int main(int argc, char **argv)
{
  mtime mt("all");
  config cfg;
  fstream fconfig("../config.md");
  string temp, inPath, savepath;
  int isamp;
  getline(fconfig, temp);
  // getline(fconfig, inPath);
  fconfig >> inPath;
  std::cout << temp << ":" << inPath << endl;
  getline(fconfig, temp);

  getline(fconfig, temp);
  fconfig >> cfg.savepath;
  std::cout << temp << ":" << cfg.savepath << endl;
  getline(fconfig, temp);

  getline(fconfig, temp);
  fconfig >> cfg.max_height_;
  std::cout << temp << ":" << cfg.max_height_ << endl;
  getline(fconfig, temp);

  getline(fconfig, temp);
  fconfig >> cfg.thread_num;
  std::cout << temp << ":" << cfg.thread_num << std::endl;
  getline(fconfig, temp);
  
  struct dirent *ptr;

  DIR *dir;
  dir = opendir(inPath.c_str());
  std::vector<string> files;
  while ((ptr = readdir(dir)) != NULL)
  {
    if (ptr->d_name[0] == '.')
      continue;
    files.push_back(string(ptr->d_name));
  }
  closedir(dir);

  threadpool executor(cfg.thread_num);
  for (int i = 0; i < files.size(); i++)
  {
    executor.commit(ground_detection, inPath, files[i], cfg);
  }
  std::cout << "files.size:" << files.size() << endl;
  executor.waitTask();
  return (0);
}
