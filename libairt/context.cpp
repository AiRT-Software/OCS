#include "net/context.h"
#include <zmqpp.hpp>


using airt::Context;


Context::Context() {
    context = std::unique_ptr<zmqpp::context>(new zmqpp::context());
}

Context::~Context() {

}