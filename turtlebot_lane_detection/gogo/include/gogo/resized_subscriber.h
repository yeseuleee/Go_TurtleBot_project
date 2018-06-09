#include <image_transport/simple_subscriber_plugin.h>
#include <gogo/ResizedImage.h>

class ResizedSubscriber : public image_transport::SimpleSubscriberPlugin<gogo::ResizedImage>
{
public:
  virtual ~ResizedSubscriber() {}

  virtual std::string getTransportName() const
  {
    return "resized";
  }

protected:
  virtual void internalCallback(const typename gogo::ResizedImage::ConstPtr& message,
                                const Callback& user_cb);
};
