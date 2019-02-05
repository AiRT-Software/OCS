#pragma once
#include <iostream>
#include <memory>

namespace restc_cpp {
    class RestClient;
}

namespace airt
{

class Rest
{
public:
    Rest();
    ~Rest();

    /**
     * @brief httpGet creates an http connection in the same thread, sends an http GET request,
     * and receives the response from the server.
     * @param reply_body will be filled with the response
     * @param endpoint URL in the server at which you will send the request
     * @return true if success when sending the request, false otherwise
     */
    bool httpGet(std::string &reply_body, const std::string & endpoint);

    /**
     * @brief httpPost creates and http conneciton in the same thread, sends an http POST request
     * with a payload, and receives the response from the server.
     * @param reply_body will be filled with the response
     * @param endpoint URL in the server at which you will send the request
     * @param payload data you want to push/post to the server
     * @return true if success when sending the request, false otherwise
     */
    bool httpPost(std::string &reply_body, const std::string & endpoint, const std::string & payload);

private:
    std::unique_ptr<restc_cpp::RestClient> rest_client;
};

};
