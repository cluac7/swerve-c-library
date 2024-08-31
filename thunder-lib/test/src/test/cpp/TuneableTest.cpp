// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>

#include <units/base.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <gtest/gtest.h> #include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <thunder/Tuneable.h>

template<typename T>
class TestErrType {
  public:
    TestErrType(bool error, T val) : error(error), val(val) {}

  private:
    bool error;
    T    val;
};

class TuneableTest : public testing::Test {
  protected:
    void SetUp() override {}

    void TearDown() override {}

    thunder::Tuneable<double> k_double =
        thunder::Tuneable<double>(1.f, "arm/tuneable/double");

    thunder::Tuneable<int> k_int =
        thunder::Tuneable<int>(1, "arm/tuneable/int");

    thunder::Tuneable<bool> k_bool =
        thunder::Tuneable<bool>(true, "arm/tuneable/bool");

    thunder::Tuneable<units::meter_t> k_meters =
        thunder::Tuneable<units::meter_t>(5_m, "arm/tuneable/meters", true);
};

TEST_F(TuneableTest, IsSubscribingDouble) {
    nt::NetworkTableInstance::GetDefault()
        .GetTopic("arm/tuneable/double")
        .GetGenericEntry()
        .SetDouble(8.5f);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    ASSERT_EQ(double(k_double), 8.5f);
}

TEST_F(TuneableTest, IsCallingDouble) {
    bool   called;
    double with = 0.0f;

    k_double | std::bind(
                   [&called, &with](double kP) -> int {
                       called = true;
                       with   = kP;
                       return 0;
                   },
                   std::placeholders::_1
               );

    called = false;

    nt::NetworkTableInstance::GetDefault()
        .GetTopic("arm/tuneable/double")
        .GetGenericEntry()
        .SetDouble(8.5f);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    ASSERT_TRUE(called);
    ASSERT_FLOAT_EQ(with, 8.5f);
}

TEST_F(TuneableTest, IsSubscribingInt) {
    nt::NetworkTableInstance::GetDefault()
        .GetTopic("arm/tuneable/int")
        .GetGenericEntry()
        .SetInteger(8);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    ASSERT_EQ(int(k_int), 8);
}

TEST_F(TuneableTest, IsCallingInt) {
    bool called = false;
    int  with;

    k_int | [&called, &with](int i) -> void {
        called = true;
        with   = i;
    };

    called = false;

    nt::NetworkTableInstance::GetDefault()
        .GetTopic("arm/tuneable/int")
        .GetGenericEntry()
        .SetInteger(8);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    ASSERT_TRUE(called);
    ASSERT_EQ(with, 8);
}

TEST_F(TuneableTest, IsSubscribingBool) {
    nt::NetworkTableInstance::GetDefault()
        .GetTopic("arm/tuneable/bool")
        .GetGenericEntry()
        .SetBoolean(false);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    ASSERT_EQ(bool(k_bool), false);
}

TEST_F(TuneableTest, IsCallingBool) {
    bool called = false;
    bool with   = true;

    k_bool | [&called, &with](bool b) -> float {
        called = true;
        with   = b;
        return 0.0f;
    };

    called = false;

    nt::NetworkTableInstance::GetDefault()
        .GetTopic("arm/tuneable/bool")
        .GetGenericEntry()
        .SetBoolean(false);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    ASSERT_TRUE(called);
    ASSERT_FALSE(with);
}

TEST_F(TuneableTest, IsSubscribingUnits) {
    nt::NetworkTableInstance::GetDefault()
        .GetTopic("arm/tuneable/meters (m)")
        .GetGenericEntry()
        .SetDouble(10);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    ASSERT_EQ(units::meter_t(k_meters), 10_m);
    ASSERT_EQ(units::inch_t(k_meters), units::inch_t(10_m));
}

TEST_F(TuneableTest, IsCallingUnits) {
    bool           called;
    units::meter_t with;

    k_meters |
        [&called, &with](units::meter_t m) -> TestErrType<units::meter_t> {
        called = true;
        with   = m;
        return TestErrType<units::meter_t>(false, m);
    };

    called = false;

    nt::NetworkTableInstance::GetDefault()
        .GetTopic("arm/tuneable/meters (m)")
        .GetGenericEntry()
        .SetDouble(10);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    ASSERT_TRUE(called);
    ASSERT_EQ(with, 10_m);
}

TEST_F(TuneableTest, IsAbbreviatingUnits) {
    auto prefixed =
        thunder::Tuneable<units::meter_t>(5_m, "/arm/tuneable/val", true);
    auto unPrefixed =
        thunder::Tuneable<units::meter_t>(7_m, "/arm/tuneable/val", false);

    double valPrefixed = nt::NetworkTableInstance::GetDefault()
                             .GetTopic("/arm/tuneable/val (m)")
                             .GetGenericEntry()
                             .GetDouble(0.0f);

    double valUnPrefixed = nt::NetworkTableInstance::GetDefault()
                               .GetTopic("/arm/tuneable/val")
                               .GetGenericEntry()
                               .GetDouble(0.0f);

    ASSERT_FLOAT_EQ(valPrefixed, 5.f);
    ASSERT_FLOAT_EQ(valUnPrefixed, 7.f);
}

ADD_COMPOUND_UNIT(
    thunder, units::volt_t {} / units::second_t {}, volts_per_second, vps
)

TEST_F(TuneableTest, IsAbbreviatingCompoundUnits) {
    auto prefixed = thunder::Tuneable<thunder::volts_per_second_t>(
        7_V / 5_s, "/arm/tuneable/val", true
    );
    auto unPrefixed = thunder::Tuneable<thunder::volts_per_second_t>(
        5_V / 5_s, "/arm/tuneable/val", false
    );

    double valPrefixed = nt::NetworkTableInstance::GetDefault()
                             .GetTopic("/arm/tuneable/val (vps)")
                             .GetGenericEntry()
                             .GetDouble(0.0f);

    double valUnPrefixed = nt::NetworkTableInstance::GetDefault()
                               .GetTopic("/arm/tuneable/val")
                               .GetGenericEntry()
                               .GetDouble(0.0f);

    ASSERT_FLOAT_EQ(valPrefixed, (7_V / 5_s).value());
    ASSERT_FLOAT_EQ(valUnPrefixed, (5_V / 5_s).value());
}

TEST_F(TuneableTest, IsCompounding) {
    bool                    called = false;
    std::tuple<double, int> with1;
    thunder::Compound(k_double, k_int) |
        [&called, &with1](double d, int i) -> void {
        called = true;
        with1  = std::make_tuple(d, i);
    };

    nt::NetworkTableInstance::GetDefault()
        .GetTopic("arm/tuneable/double")
        .GetGenericEntry()
        .SetDouble(10);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    ASSERT_TRUE(called);
    ASSERT_FLOAT_EQ(k_double, 10);
    ASSERT_FLOAT_EQ(std::get<0>(with1), double(k_double));
    ASSERT_EQ(std::get<1>(with1), int(k_int));

    called = false;
    std::tuple<units::meter_t, double, int> with2;
    thunder::Compound(k_meters, k_int, k_double) |
        [&called, &with2](units::meter_t m, int i, double d) -> void {
        called = true;
        with2  = std::make_tuple(m, d, i);
    };

    nt::NetworkTableInstance::GetDefault()
        .GetTopic("arm/tuneable/meters (m)")
        .GetGenericEntry()
        .SetDouble(20);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    ASSERT_TRUE(called);
    ASSERT_EQ(units::meter_t(k_meters), 20_m);
    ASSERT_EQ(std::get<0>(with2), units::meter_t(k_meters));
    ASSERT_FLOAT_EQ(std::get<1>(with2), double(k_double));
    ASSERT_EQ(std::get<2>(with2), int(k_int));
}
