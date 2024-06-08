#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace cargo {
namespace system_monitor {

class ProcFile {
 public:
  explicit ProcFile(std::string filename) : name_(filename), fs_(name_) {}
  virtual ~ProcFile() { fs_.close(); }

  bool ReadOneLine(std::vector<std::string> &all_segs) {
    std::string line;
    std::getline(fs_, line);

    if (fs_.eof() || line.size() == 0) {
      return false;
    }

    std::istringstream ss(line);
    while (!ss.eof()) {
      std::string word;
      std::string seg;
      ss >> seg;
      if (seg[0] == '(') {
        word += seg;
        while (seg[seg.size() - 1] != ')') {
          seg.clear();
          ss >> seg;
          word += seg;
        }
        all_segs.push_back(word);
      } else {
        all_segs.push_back(seg);
        continue;
      }
    }

    return true;
  }

  bool ReadOneWord(std::istringstream &line) { return true; }

  template <class... Args>
  bool ReadOneLine(Args &... args) {
    std::string line;
    std::getline(fs_, line);

    if (fs_.eof() || line.empty()) {
      return false;
    }

    std::istringstream ss(line);
    return ReadOneWord(ss, args...);
  }

 private:
  template <class T, class... Args>
  bool ReadOneWord(std::istringstream &line, T &head, Args &... args) {
    if (line.eof()) return false;

    line >> head;
    return ReadOneWord(line, args...);
  }
  std::string name_;
  std::ifstream fs_;
};
}  // namespace system_monitor
}  // namespace cargo
