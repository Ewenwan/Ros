
template<class Msg>
class Callback {
public:
  Callback() : times_called(0)
  {
  }

  void callback(const Msg msg)
  {
    times_called++;
    last_msg = msg;
  }

  Msg last_msg;
  int times_called;
};
typedef Callback<std_msgs::String> StringCallback;


