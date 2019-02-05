#pragma once

#ifndef RESTC_CPP_IO_TIMER_H_
#define RESTC_CPP_IO_TIMER_H_

#include <iostream>

#include <boost/utility/string_ref.hpp>

#include "restc-cpp/restc-cpp.h"
#include "restc-cpp/logging.h"
#include "restc-cpp/Socket.h"
#include "restc-cpp/Connection.h"

namespace restc_cpp {

class IoTimer : public std::enable_shared_from_this<IoTimer>
{
public:
    using ptr_t = std::shared_ptr<IoTimer>;
    using close_t = std::function<void ()>;

    class Wrapper
    {
        public:
            Wrapper(ptr_t&& timer)
            : timer_{std::move(timer)} {}

            Wrapper() {}

        ~Wrapper() {
            if (timer_) {
                timer_->Cancel();
            }
        }

        void Cancel() {
            if (timer_) {
                timer_->Cancel();
            }
        }

        private:
            ptr_t timer_ = nullptr;
    };

    using wrapper_t = std::unique_ptr<Wrapper>;

    ~IoTimer() {
    }

    void Handler(const boost::system::error_code& error) {

        if (error) {
            return;
        }

        if (is_active_) {
            is_active_ = false;
            is_expiered_ = true;
            close_();
        }
    }

    void Cancel() {
        if (is_active_) {
            is_active_ = false;
            RESTC_CPP_LOG_TRACE << "Canceled timer " << timer_name_;
        }
    }

    bool IsExpiered() const noexcept { return is_expiered_; }

    static ptr_t Create(
        const std::string& timerName,
        int milliseconds_timeout,
        boost::asio::io_service& io_service,
        close_t close) {

        ptr_t timer;
        // Private constructor, we cannot use std::make_shared()
        timer.reset(new IoTimer(timerName, io_service, close));
        timer->Start(milliseconds_timeout);
        return timer;
    }

    /*! Convenience factory.
     *
     * Creates an instance of a timer, wrapped in unique_ptr
     * that will cancel the timer when the wrapper goes
     * out of scope.
     *
     * \param connection Connection to watch. If the timer expires
     *      before cancel is called, the connection is closed.
     */
    static wrapper_t Create(const std::string& timerName,
                        int milliseconds_timeout,
                        const Connection::ptr_t& connection) {

        if (!connection || (milliseconds_timeout <= 0)) {
            return std::make_unique<Wrapper>();
        }

        std::weak_ptr<Connection> weak_connection = connection;

        RESTC_CPP_LOG_TRACE << "Created timer " << timerName
            << " for " << *connection;

        return std::make_unique<Wrapper>(Create(
            timerName,
            milliseconds_timeout,
            connection->GetSocket().GetSocket().get_io_service(),
            [weak_connection, timerName]() {
                if (auto connection = weak_connection.lock()) {
                    if (connection->GetSocket().GetSocket().is_open()) {
                        RESTC_CPP_LOG_TRACE
                            << "Timer " << timerName << ": "
                            << *connection
                            << " timed out.";
                        connection->GetSocket().Close(Socket::Reason::TIME_OUT);
                    }
                }
            }));
    }


private:
    IoTimer(const std::string& timerName, boost::asio::io_service& io_service,
            close_t close)
    : close_{close}, timer_{io_service}, timer_name_{timerName}
    {}

    void Start(int millisecondsTimeOut)
    {
        timer_.expires_from_now(
            boost::posix_time::milliseconds(millisecondsTimeOut));
        is_active_ = true;
        try {
            timer_.async_wait(std::bind(
                &IoTimer::Handler,
                shared_from_this(),
                std::placeholders::_1));
        } catch (const std::exception&) {
            is_active_ = false;
        }
    }

private:
    bool is_active_ = false;
    bool is_expiered_ = false;
    close_t close_;
    boost::asio::deadline_timer timer_;
    const std::string timer_name_;
};

} // restc_cpp

#endif // RESTC_CPP_IO_TIMER_H_

