#include "async_web_server_cpp/http_reply.hpp"

#include <boost/bind/bind.hpp>
#include <boost/make_shared.hpp>

namespace async_web_server_cpp
{

HttpConnection::HttpConnection(boost::asio::io_service& io_service,
                               HttpServerRequestHandler handler)
    : strand_(io_service), socket_(io_service), request_handler_(handler),
      write_in_progress_(false)
{
}

boost::asio::ip::tcp::socket& HttpConnection::socket()
{
    return socket_;
}

void HttpConnection::start()
{
    async_read(
        boost::bind(&HttpConnection::handle_read, shared_from_this(), boost::placeholders::_1, boost::placeholders::_2));
}

void HttpConnection::handle_read(const char* begin, const char* end)
{
    boost::tribool result;
    const char* parse_end;
    boost::tie(result, parse_end) = request_parser_.parse(request_, begin, end);

    if (result)
    {
        request_.parse_uri();
        try
        {
            request_handler_(request_, shared_from_this(), parse_end, end);
        }
        catch (...)
        {
            // error constructing request
            // just kill the connection as the handler may have already started
            // writing stuff out
        }
    }
    else if (!boost::logic::indeterminate(result))
    {
        HttpReply::stock_reply(HttpReply::bad_request)(
            request_, shared_from_this(), begin, end);
    }
    else
    {
        async_read(boost::bind(&HttpConnection::handle_read, shared_from_this(),
                               boost::placeholders::_1, boost::placeholders::_2));
    }
}

void HttpConnection::handle_read_raw(ReadHandler callback,
                                     const boost::system::error_code& e,
                                     std::size_t bytes_transferred)
{
    if (!e)
    {
        callback(buffer_.data(), buffer_.data() + bytes_transferred);
    }
    else
    {
        last_error_ = e;
    }
}
void HttpConnection::async_read(ReadHandler callback)
{
    if (last_error_)
    {
        boost::throw_exception(boost::system::system_error(last_error_));
    }
    socket_.async_read_some(
        boost::asio::buffer(buffer_),
        strand_.wrap(
            boost::bind(&HttpConnection::handle_read_raw, shared_from_this(),
                        callback, boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred)));
}

void HttpConnection::write_and_clear(std::vector<unsigned char>& data)
{
    boost::shared_ptr<std::vector<unsigned char>> buffer(
        new std::vector<unsigned char>());
    buffer->swap(data);
    std::vector<boost::asio::const_buffer> buffers;
    buffers.push_back(boost::asio::buffer(*buffer));
    std::vector<ResourcePtr> resources;
    resources.push_back(buffer);
    enqueue_write(std::move(buffers), std::move(resources), false);
}

void HttpConnection::write(const std::string& content)
{
    boost::shared_ptr<std::string> str(new std::string(content));
    std::vector<boost::asio::const_buffer> buffers;
    buffers.push_back(boost::asio::buffer(*str));
    std::vector<ResourcePtr> resources;
    resources.push_back(str);
    enqueue_write(std::move(buffers), std::move(resources), false);
}

void HttpConnection::write(const boost::asio::const_buffer& buffer,
                           ResourcePtr resource)
{
    std::vector<boost::asio::const_buffer> buffers;
    buffers.push_back(buffer);
    std::vector<ResourcePtr> resources;
    if (resource)
        resources.push_back(resource);
    enqueue_write(std::move(buffers), std::move(resources), false);
}

void HttpConnection::write(
    const std::vector<boost::asio::const_buffer>& buffers, ResourcePtr resource)
{
    std::vector<ResourcePtr> resources;
    if (resource)
        resources.push_back(resource);
    enqueue_write(buffers, std::move(resources), false);
}

void HttpConnection::write_replaceable(
    const std::vector<boost::asio::const_buffer>& buffers, ResourcePtr resource)
{
    std::vector<ResourcePtr> resources;
    if (resource)
        resources.push_back(resource);
    enqueue_write(buffers, std::move(resources), true);
}

void HttpConnection::enqueue_write(std::vector<boost::asio::const_buffer> buffers,
                                   std::vector<ResourcePtr> resources,
                                   bool replaceable)
{
    boost::mutex::scoped_lock lock(write_mutex_);
    if (replaceable)
    {
        for (auto it = pending_writes_.begin(); it != pending_writes_.end();)
        {
            if (it->replaceable)
            {
                it = pending_writes_.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }
    pending_writes_.push_back(PendingWrite{std::move(buffers), std::move(resources), replaceable});
    if (!write_in_progress_)
        write_pending();
}

// Must be called while holding write lock
void HttpConnection::write_pending()
{
    if (last_error_)
    {
        boost::throw_exception(boost::system::system_error(last_error_));
    }
    if (pending_writes_.empty())
    {
        write_in_progress_ = false;
        return;
    }

    write_in_progress_ = true;
    PendingWrite pending_write = std::move(pending_writes_.front());
    pending_writes_.pop_front();
    boost::asio::async_write(socket_, pending_write.buffers,
                             boost::bind(&HttpConnection::handle_write,
                                         shared_from_this(),
                                         boost::asio::placeholders::error,
                                         pending_write.resources));
}

void HttpConnection::handle_write(const boost::system::error_code& e,
                                  std::vector<ResourcePtr> resources)
{
    boost::mutex::scoped_lock lock(write_mutex_);
    write_in_progress_ = false;
    if (!e)
    {
        if (!pending_writes_.empty())
        {
            write_pending();
        }
    }
    else
    {
        last_error_ = e;
    }
}

}  // namespace async_web_server_cpp
