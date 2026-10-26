#include <gtest/gtest.h>
#include <cmath>

#include "bumperbot_firmware/serial_protocol.hpp"

using bumperbot_firmware::find_wheel_index;
using bumperbot_firmware::format_wheel_command;
using bumperbot_firmware::parse_wheel_message;
using bumperbot_firmware::take_last_line;
using bumperbot_firmware::WheelIntegrator;

TEST(SerialProtocol, ParsesBothWheels)
{
  auto r = parse_wheel_message("rp05.20,ln03.10,");
  EXPECT_TRUE(r.has_right);
  EXPECT_TRUE(r.has_left);
  EXPECT_DOUBLE_EQ(r.right, 5.2);
  EXPECT_DOUBLE_EQ(r.left, -3.1);
  EXPECT_EQ(r.rejected, 0);
}

TEST(SerialProtocol, IgnoresLineEndingsAndOrder)
{
  auto r = parse_wheel_message("ln12.50,rp00.00,\r\n");
  EXPECT_DOUBLE_EQ(r.left, -12.5);
  EXPECT_DOUBLE_EQ(r.right, 0.0);
  EXPECT_EQ(r.rejected, 0);
}

TEST(SerialProtocol, CorruptedTokensDoNotThrow)
{
  for (const std::string bad : {"r", "rp", "x", "rp,", "rq05.0,", "rp05.2x,", "lnabc,", ",,,", "", "rpnan,", "\xff\xfe"})
  {
    EXPECT_NO_THROW({
      auto r = parse_wheel_message(bad);
      EXPECT_FALSE(r.has_right) << bad;
      EXPECT_FALSE(r.has_left) << bad;
    });
  }
  auto partial = parse_wheel_message("p05.20,ln03.10,");
  EXPECT_FALSE(partial.has_right);
  EXPECT_TRUE(partial.has_left);
  EXPECT_EQ(partial.rejected, 1);
}

TEST(SerialProtocol, FormatsCommandsLikeTheFirmwareExpects)
{
  EXPECT_EQ(format_wheel_command(5.2, -3.1), "rp05.20,ln03.10,");
  EXPECT_EQ(format_wheel_command(-12.346, 0.0), "rn12.35,lp00.00,");
}

TEST(SerialProtocol, FormatThenParseRoundTrip)
{
  auto r = parse_wheel_message(format_wheel_command(-4.25, 7.5));
  EXPECT_DOUBLE_EQ(r.right, -4.25);
  EXPECT_DOUBLE_EQ(r.left, 7.5);
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
    EXPECT_FALSE(r.has_right) << bad;
    EXPECT_EQ(r.rejected, 1) << bad;
  }
}

TEST(SerialProtocol, TakeLastLineKeepsTheUnfinishedTail)
{
  std::string buffer = "rp01.00,lp01.00,\r\nrp02.00,lp02.00,\r\nrp03";
  std::string line;
  ASSERT_TRUE(take_last_line(buffer, line));
  EXPECT_EQ(line, "rp02.00,lp02.00,\r");
  EXPECT_EQ(buffer, "rp03");
  EXPECT_FALSE(take_last_line(buffer, line));
  buffer += ".00,lp03.00,\r\n";
  ASSERT_TRUE(take_last_line(buffer, line));
  EXPECT_EQ(parse_wheel_message(line).right, 3.0);
  EXPECT_TRUE(buffer.empty());
}

TEST(SerialProtocol, TakeLastLineDropsGarbageWithoutNewline)
{
  std::string buffer(300, 'x');
  std::string line;
  EXPECT_FALSE(take_last_line(buffer, line));
  EXPECT_TRUE(buffer.empty());
  buffer = "\nrp01.00,lp01.00,\n";
  ASSERT_TRUE(take_last_line(buffer, line));
  EXPECT_EQ(line, "rp01.00,lp01.00,");
}

TEST(WheelIntegrator, HoldsTheLastSpeedBetweenReadings)
{
  WheelIntegrator w(0.3);
  w.on_reading(0.0, parse_wheel_message("rp02.00,ln01.00,"));
  double t = 0.0;
  for (int i = 0; i < 10; ++i)
  {
    t += 0.01;
    EXPECT_FALSE(w.advance(t, 0.01));
  }
  EXPECT_NEAR(w.right_position(), 0.2, 1e-9);
  EXPECT_NEAR(w.left_position(), -0.1, 1e-9);
}

TEST(WheelIntegrator, StopsWhenTheArduinoGoesQuiet)
{
  WheelIntegrator w(0.3);
  w.on_reading(0.0, parse_wheel_message("rp05.00,lp05.00,"));
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
  EXPECT_NEAR(w.right_position(), 5.0 * 0.3, 0.06);
  w.on_reading(t, parse_wheel_message("rp01.00,lp01.00,"));
  EXPECT_FALSE(w.advance(t + 0.01, 0.01));
  EXPECT_DOUBLE_EQ(w.right_velocity(), 1.0);
}

TEST(WheelIntegrator, GarbageDoesNotKeepTheDataAlive)
{
  WheelIntegrator w(0.3);
  w.on_reading(0.0, parse_wheel_message("rp05.00,lp05.00,"));
  for (int i = 1; i <= 100; ++i)
  {
    w.on_reading(i * 0.01, parse_wheel_message("xx,rq,"));
  }
  EXPECT_TRUE(w.advance(1.0, 0.01));
}

TEST(WheelIntegrator, NoMotionBeforeTheFirstReading)
{
  WheelIntegrator w(0.3);
  EXPECT_TRUE(w.advance(0.5, 0.01));
  EXPECT_DOUBLE_EQ(w.right_position(), 0.0);
}

TEST(WheelIntegrator, OneWheelReadingsDoNotKeepTheDataAlive)
{
  WheelIntegrator w(0.3);
  w.on_reading(0.0, parse_wheel_message("rp05.00,lp05.00,"));
  for (int i = 1; i <= 100; ++i)
  {
    w.on_reading(i * 0.01, parse_wheel_message("rp05.00,"));
  }
  EXPECT_TRUE(w.advance(1.0, 0.01));
  EXPECT_DOUBLE_EQ(w.left_velocity(), 0.0);
  EXPECT_TRUE(w.has_reading());
}

TEST(WheelIntegrator, ForgetReadingKeepsThePosition)
{
  WheelIntegrator w(0.3);
  w.on_reading(0.0, parse_wheel_message("rp05.00,ln02.00,"));
  w.advance(0.1, 0.1);
  w.forget_reading();
  EXPECT_FALSE(w.has_reading());
  EXPECT_DOUBLE_EQ(w.right_velocity(), 0.0);
  EXPECT_NEAR(w.right_position(), 0.5, 1e-9);
  EXPECT_NEAR(w.left_position(), -0.2, 1e-9);
  EXPECT_TRUE(w.advance(0.2, 0.1));
  EXPECT_NEAR(w.right_position(), 0.5, 1e-9);
  w.on_reading(0.3, parse_wheel_message("rp01.00,lp01.00,"));
  EXPECT_FALSE(w.advance(0.4, 0.1));
  EXPECT_NEAR(w.right_position(), 0.6, 1e-9);
}

TEST(SerialProtocol, CommandIsAlwaysReadableByTheSketch)
{
  EXPECT_EQ(format_wheel_command(std::nan(""), 1.0), "rp00.00,lp01.00,");
  EXPECT_EQ(format_wheel_command(INFINITY, -INFINITY), "rp00.00,lp00.00,");
  EXPECT_EQ(format_wheel_command(150.0, -150.0), "rp99.99,ln99.99,");
}
