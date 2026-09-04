// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#ifndef _TIC_TOC_H_
#define _TIC_TOC_H_

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iostream>

/**
 * @brief Simple timer utility class based on std::chrono.
 */
class TicToc {
public:
    /**
     * @brief Construct a new TicToc timer and start it.
     */
    TicToc() { tic(); }

    /**
     * @brief Start or reset the timer.
     */
    void tic() { start = std::chrono::system_clock::now(); }

    /**
     * @brief Get elapsed time in milliseconds.
     * @return Elapsed time in milliseconds since the last tic().
     */
    double toc() {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

/**
 * @brief Version 2 of TicToc timer with automated printing options.
 */
class TicTocV2 {
public:
    /**
     * @brief Construct a new TicTocV2 timer.
     */
    TicTocV2() { tic(); }

    /**
     * @brief Construct a new TicTocV2 timer with print flag.
     * @param _disp Whether to display results automatically on destruction/toc.
     */
    TicTocV2(bool _disp) {
        disp_ = _disp;
        tic();
    }

    /**
     * @brief Start or reset the timer.
     */
    void tic() { start = std::chrono::system_clock::now(); }

    /**
     * @brief Get elapsed time in milliseconds and print it if enabled.
     * @param _about_task Description of the task timed.
     */
    void toc(std::string _about_task) {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        double elapsed_ms = elapsed_seconds.count() * 1000;

        if (disp_) {
            std::cout.precision(3);  // 10 for sec, 3 for ms
            std::cout << _about_task << ": " << elapsed_ms << " msec." << std::endl;
        }
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
    bool disp_ = false;
};

#endif  // _TIC_TOC_H_