#include <gtest/gtest.h>
#include <cmath>
#include <string>

#include "bumperbot_firmware/serial_protocol.hpp"

using bumperbot_firmware::find_wheel_index;
using bumperbot_firmware::format_wheel_command;
using bumperbot_firmware::parse_wheel_message;
using bumperbot_firmware::take_last_line;
using bumperbot_firmware::WheelIntegrator;

namespace
{
// A line as the sketch sends it: speeds in rad/s and the encoder counters
std::string line(double right, double left, unsigned right_count, unsigned left_count)
{
  auto speed = [](char wheel, double v) {
    return std::string(1, wheel) + (v < 0 ? "n" : "p") + std::to_string(std::abs(v));
  };
  return speed('r', right) + "," + speed('l', left) + ",rc" + std::to_string(right_count) +
         ",lc" + std::to_string(left_count) + ",";
}

// 0.01 rad per count keeps the expected positions easy to read
constexpr double kRadPerCount = 0.01;
}  // namespace

TEST(SerialProtocol, ParsesBothWheels)
{
  auto r = parse_wheel_message("rp4.55,ln03.10,rc100,lc65535,");
  EXPECT_TRUE(r.complete());
  EXPECT_DOUBLE_EQ(r.right.speed, 4.55);
  EXPECT_DOUBLE_EQ(r.left.speed, -3.1);
  EXPECT_EQ(r.right.count, 100);
  EXPECT_EQ(r.left.count, 65535);
  EXPECT_EQ(r.rejected, 0);
}

TEST(SerialProtocol, IgnoresLineEndingsAndOrder)
{
  auto r = parse_wheel_message("lc5,ln12.50,rc7,rp00.00,\r\n");
  EXPECT_TRUE(r.complete());
  EXPECT_DOUBLE_EQ(r.left.speed, -12.5);
  EXPECT_DOUBLE_EQ(r.right.speed, 0.0);
  EXPECT_EQ(r.right.count, 7);
  EXPECT_EQ(r.rejected, 0);
}

TEST(SerialProtocol, CorruptedTokensDoNotThrow)
{
  for (const std::string bad : {"r", "rp", "x", "rp,", "rq05.0,", "rp05.2x,", "lnabc,", ",,,", "", "rpnan,",
                                "rc", "rc-1,", "\xff\xfe"})
  {
    EXPECT_NO_THROW({
      auto r = parse_wheel_message(bad);
      EXPECT_FALSE(r.right.has_speed || r.right.has_count) << bad;
      EXPECT_FALSE(r.left.has_speed || r.left.has_count) << bad;
    });
  }
  auto partial = parse_wheel_message("p05.20,ln03.10,");
  EXPECT_FALSE(partial.right.has_speed);
  EXPECT_TRUE(partial.left.has_speed);
  EXPECT_FALSE(partial.complete());
  EXPECT_EQ(partial.rejected, 1);
}

TEST(SerialProtocol, CountsMustBePlainIntegersUpTo65535)
{
  for (const std::string bad : {"rc-1,", "rc1.5,", "rc65536,", "rc123456,", "rcx,", "rc+5,"})
  {
    auto r = parse_wheel_message(bad);
    EXPECT_FALSE(r.right.has_count) << bad;
    EXPECT_EQ(r.rejected, 1) << bad;
  }
  EXPECT_EQ(parse_wheel_message("rc65535,").right.count, 65535);
  EXPECT_EQ(parse_wheel_message("rc0,").right.count, 0);
}

TEST(SerialProtocol, FormatsCommandsLikeTheFirmwareExpects)
{
  EXPECT_EQ(format_wheel_command(5.2, -3.1), "rp05.20,ln03.10,");
  EXPECT_EQ(format_wheel_command(-12.346, 0.0), "rn12.35,lp00.00,");
}

TEST(SerialProtocol, FormatThenParseRoundTrip)
{
  auto r = parse_wheel_message(format_wheel_command(-4.25, 7.5));
  EXPECT_DOUBLE_EQ(r.right.speed, -4.25);
  EXPECT_DOUBLE_EQ(r.left.speed, 7.5);
}

TEST(SerialProtocol, CommandIsAlwaysReadableByTheSketch)
{
  EXPECT_EQ(format_wheel_command(std::nan(""), 1.0), "rp00.00,lp01.00,");
  EXPECT_EQ(format_wheel_command(INFINITY, -INFINITY), "rp00.00,lp00.00,");
  EXPECT_EQ(format_wheel_command(150.0, -150.0), "rp99.99,ln99.99,");
}

TEST(SerialProtocol, WheelsAreFoundByNameNotByPosition)
{
  std::vector<std::string> urdf_order = {"wheel_left_joint", "wheel_right_joint"};
  EXPECT_EQ(find_wheel_index(urdf_order, "right"), 1);
  EXPECT_EQ(find_wheel_index(urdf_order, "left"), 0);
  std::vector<std::string> swapped = {"wheel_right_joint", "wheel_left_joint"};
  EXPECT_EQ(find_wheel_index(swapped, "right"), 0);
  EXPECT_EQ(find_wheel_index(swapped, "left"), 1);
  std::vector<std::string> other_names = {"left_wheel", "right_wheel"};
  EXPECT_EQ(find_wheel_index(other_names, "right"), 1);
  EXPECT_EQ(find_wheel_index({"a", "b"}, "right"), -1);
}

TEST(SerialProtocol, NumbersMustBePlainDecimals)
{
  for (const std::string bad : {"rp-5.00,", "rp0x1A,", "rp+3.0,", "rp1e3,", "rpnan,"})
  {
    auto r = parse_wheel_message(bad);
    EXPECT_FALSE(r.right.has_speed) << bad;
    EXPECT_EQ(r.rejected, 1) << bad;
  }
}

TEST(SerialProtocol, TakeLastLineKeepsTheUnfinishedTail)
{
  std::string buffer = "rp01.00,lp01.00,\r\nrp02.00,lp02.00,\r\nrp03";
  std::string text;
  ASSERT_TRUE(take_last_line(buffer, text));
  EXPECT_EQ(text, "rp02.00,lp02.00,\r");
  EXPECT_EQ(buffer, "rp03");
  EXPECT_FALSE(take_last_line(buffer, text));
  buffer += ".00,lp03.00,\r\n";
  ASSERT_TRUE(take_last_line(buffer, text));
  EXPECT_EQ(parse_wheel_message(text).right.speed, 3.0);
  EXPECT_TRUE(buffer.empty());
}

TEST(SerialProtocol, TakeLastLineDropsGarbageWithoutNewline)
{
  std::string buffer(300, 'x');
  std::string text;
  EXPECT_FALSE(take_last_line(buffer, text));
  EXPECT_TRUE(buffer.empty());
  buffer = "\nrp01.00,lp01.00,\n";
  ASSERT_TRUE(take_last_line(buffer, text));
  EXPECT_EQ(text, "rp01.00,lp01.00,");
}

TEST(WheelIntegrator, PositionComesFromTheCounters)
{
  WheelIntegrator w(0.3, kRadPerCount);
  w.on_reading(0.0, parse_wheel_message(line(1.0, -1.0, 1000, 1000)));
  EXPECT_DOUBLE_EQ(w.right_position(), 0.0);
  w.on_reading(0.1, parse_wheel_message(line(1.0, -1.0, 1100, 900)));
  EXPECT_NEAR(w.right_position(), 1.0, 1e-9);
  EXPECT_NEAR(w.left_position(), -1.0, 1e-9);
}

TEST(WheelIntegrator, MovesWithTheLastSpeedBetweenReadingsAndSnapsBack)
{
  WheelIntegrator w(0.3, kRadPerCount);
  w.on_reading(0.0, parse_wheel_message(line(2.0, -1.0, 0, 0)));
  double t = 0.0;
  for (int i = 0; i < 10; ++i)
  {
    t += 0.01;
    EXPECT_FALSE(w.advance(t, 0.01));
  }
  EXPECT_NEAR(w.right_position(), 0.2, 1e-9);
  EXPECT_NEAR(w.left_position(), -0.1, 1e-9);
  w.on_reading(t, parse_wheel_message(line(2.0, -1.0, 19, 65526)));
  EXPECT_NEAR(w.right_position(), 0.19, 1e-9);
  EXPECT_NEAR(w.left_position(), -0.10, 1e-9);
}

TEST(WheelIntegrator, LostLinesDoNotLoseDistance)
{
  WheelIntegrator w(0.3, kRadPerCount);
  w.on_reading(0.0, parse_wheel_message(line(1.0, 1.0, 0, 0)));
  w.on_reading(0.2, parse_wheel_message(line(1.0, 1.0, 200, 200)));
  EXPECT_NEAR(w.right_position(), 2.0, 1e-9);
}

TEST(WheelIntegrator, CounterWrapsAround)
{
  WheelIntegrator w(0.3, kRadPerCount);
  w.on_reading(0.0, parse_wheel_message(line(1.0, -1.0, 65530, 10)));
  w.on_reading(0.1, parse_wheel_message(line(1.0, -1.0, 10, 65530)));
  EXPECT_NEAR(w.right_position(), 0.16, 1e-9);
  EXPECT_NEAR(w.left_position(), -0.16, 1e-9);
}

TEST(WheelIntegrator, StopsWhenTheArduinoGoesQuiet)
{
  WheelIntegrator w(0.3, kRadPerCount);
  w.on_reading(0.0, parse_wheel_message(line(5.0, 5.0, 0, 0)));
  double t = 0.0;
  bool stale = false;
  for (int i = 0; i < 150; ++i)
  {
    t += 0.01;
    stale = w.advance(t, 0.01);
  }
  EXPECT_TRUE(stale);
  EXPECT_DOUBLE_EQ(w.right_velocity(), 0.0);
  EXPECT_DOUBLE_EQ(w.left_velocity(), 0.0);
  EXPECT_DOUBLE_EQ(w.right_position(), 0.0);
  w.on_reading(t, parse_wheel_message(line(1.0, 1.0, 150, 150)));
  EXPECT_FALSE(w.advance(t + 0.01, 0.01));
  EXPECT_DOUBLE_EQ(w.right_velocity(), 1.0);
  EXPECT_NEAR(w.right_position(), 1.51, 1e-9);
}

TEST(WheelIntegrator, GarbageDoesNotKeepTheDataAlive)
{
  WheelIntegrator w(0.3, kRadPerCount);
  w.on_reading(0.0, parse_wheel_message(line(5.0, 5.0, 0, 0)));
  for (int i = 1; i <= 100; ++i)
  {
    w.on_reading(i * 0.01, parse_wheel_message("xx,rq,"));
  }
  EXPECT_TRUE(w.advance(1.0, 0.01));
}

TEST(WheelIntegrator, NoMotionBeforeTheFirstReading)
{
  WheelIntegrator w(0.3, kRadPerCount);
  EXPECT_TRUE(w.advance(0.5, 0.01));
  EXPECT_DOUBLE_EQ(w.right_position(), 0.0);
}

TEST(WheelIntegrator, IncompleteReadingsDoNotKeepTheDataAlive)
{
  WheelIntegrator w(0.3, kRadPerCount);
  w.on_reading(0.0, parse_wheel_message(line(5.0, 5.0, 0, 0)));
  for (int i = 1; i <= 100; ++i)
  {
    w.on_reading(i * 0.01, parse_wheel_message("rp05.00,rc10,"));
    w.on_reading(i * 0.01, parse_wheel_message("rp05.00,lp05.00,"));
  }
  EXPECT_TRUE(w.advance(1.0, 0.01));
  EXPECT_DOUBLE_EQ(w.left_velocity(), 0.0);
  EXPECT_TRUE(w.has_reading());
}

TEST(WheelIntegrator, ForgetReadingKeepsThePositionAndTakesANewReference)
{
  WheelIntegrator w(0.3, kRadPerCount);
  w.on_reading(0.0, parse_wheel_message(line(5.0, -2.0, 0, 0)));
  w.on_reading(0.1, parse_wheel_message(line(5.0, -2.0, 50, 65516)));
  w.advance(0.15, 0.05);
  w.forget_reading();
  EXPECT_FALSE(w.has_reading());
  EXPECT_DOUBLE_EQ(w.right_velocity(), 0.0);
  EXPECT_NEAR(w.right_position(), 0.5, 1e-9);
  EXPECT_NEAR(w.left_position(), -0.2, 1e-9);
  EXPECT_TRUE(w.advance(0.2, 0.1));
  EXPECT_NEAR(w.right_position(), 0.5, 1e-9);
  // The Arduino restarted: its counters begin again at 0
  w.on_reading(0.3, parse_wheel_message(line(1.0, 1.0, 0, 0)));
  EXPECT_NEAR(w.right_position(), 0.5, 1e-9);
  w.on_reading(0.4, parse_wheel_message(line(1.0, 1.0, 10, 10)));
  EXPECT_NEAR(w.right_position(), 0.6, 1e-9);
  EXPECT_NEAR(w.left_position(), -0.1, 1e-9);
}
