/*
 * IC-GVINS: A Robust, Real-time, INS-Centric GNSS-Visual-Inertial Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef FILEBASE_H
#define FILEBASE_H

#include <fstream>
#include <vector>

using std::string;
using std::vector;

class FileBase {

public:
    // 为文件定义枚举类，文本文件为0，二进制文件为1
    enum FileType {
        TEXT   = 0,
        BINARY = 1,
    };

    FileBase() = default; // 使用系统提供的默认构造函数
    ~FileBase() { // 析构函数将文件关闭
        if (isOpen()) 
            filefp_.close();
    }

    void close() { 
        filefp_.close();
    }

    bool isOpen() {
        return filefp_.is_open();
    }

    bool isEof() { // 判断是否到达文件末尾
        return filefp_.eof();
    }

    std::fstream &fstream() { // 返回文件流本身
        return filefp_;
    }

    int columns() const { // 返回文件中文字的列数
        return columns_;
    }
    
    // 清空缓冲区，本质上是将缓冲区中的内容立即输出到文件/屏幕中
    void flush() {
        filefp_.flush();
    }

protected:
    std::fstream filefp_;
    int filetype_ = TEXT;

    int columns_ = 0;
};

#endif // FILEBASE_H
