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

#ifndef GPS_TIME_H
#define GPS_TIME_H

#include <cmath>

// GPS is now ahead of UTC by 18 seconds
#define GPS_LEAP_SECOND 18

/**
 * UNIX时间戳：UTC时间都是从1970年01月01日0:00:00开始计算秒数的，这个描述就是unix时间戳
 * gps时间戳：GPS始于1980年1月6日，连续时间增加不调秒
 * unix_timestamp = gps_timestamp + 两个时间起始相差的固定秒数 - GPS_LEAP_SECOND
 * GPS_LEAP_SECOND为目前为止，GPS超前UTC时间18秒。
 */



class GpsTime {

public:
    static void gps2unix(int week, double sow, double &unixs) {
        unixs = sow + week * 604800 + 315964800 - GPS_LEAP_SECOND; // 6048为一周的时间秒数
    };
    
    static void unix2gps(double unixs, int &week, double &sow) {
        double seconds = unixs + GPS_LEAP_SECOND - 315964800; // seconds为GPS的时间戳（从1980年1月6日计起）

        week = floor(seconds / 604800); // 得到周时
        sow  = seconds - week * 604800; // sow(Seconds of Week)表示周内秒。是指从当前GPS周的起始时刻（通常为周日零点）到当前时间的总秒数
    };
};

#endif // GPS_TIME_H
