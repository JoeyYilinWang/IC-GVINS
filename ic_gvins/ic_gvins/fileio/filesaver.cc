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

#include "fileio/filesaver.h"

#include <absl/strings/str_format.h>

FileSaver::FileSaver(const string &filename, int columns, int filetype) {
    open(filename, columns, filetype);
}

bool FileSaver::open(const string &filename, int columns, int filetype) {
    auto type = filetype == TEXT ? std::ios_base::out : (std::ios_base::out | std::ios_base::binary);
    filefp_.open(filename, type);

    columns_  = columns;
    filetype_ = filetype;

    return isOpen();
}

// 将data数据填充整行
void FileSaver::dump(const vector<double> &data) {
    dump_(data);
}

// 将data数据填充多行
void FileSaver::dumpn(const vector<vector<double>> &data) {
    for (const auto &k : data) {
        dump_(k);
    }
}

void FileSaver::dump_(const vector<double> &data) {
    if (filetype_ == TEXT) {
        string line;

        // 定义字符串格式
        constexpr absl::string_view format = "%-15.9lf ";

        // 初始化行首值
        line = absl::StrFormat(format, data[0]);
        // 填充整行
        for (size_t k = 1; k < data.size(); k++) {
            absl::StrAppendFormat(&line, format, data[k]);
        }
        // 换行
        filefp_ << line << "\n";
    } else {
        // 将data中的所有数据以二进制的形式写入到文件中
        filefp_.write(reinterpret_cast<const char *>(data.data()), sizeof(double) * data.size());
    }
}

FileSaver::~FileSaver() {
    if (isOpen()) {
        flush(); // 清理缓冲区
        close();
    }
}
