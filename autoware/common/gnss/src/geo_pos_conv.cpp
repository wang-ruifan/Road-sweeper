/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gnss/geo_pos_conv.hpp>

#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

geo_pos_conv::geo_pos_conv()
    : m_x(0)
    , m_y(0)
    , m_z(0)
    , m_lat(0)
    , m_lon(0)
    , m_h(0)
    , m_PLato(0)
    , m_PLo(0)
{
}

double geo_pos_conv::x() const
{
  return m_x;
}

double geo_pos_conv::y() const
{
  return m_y;
}

double geo_pos_conv::z() const
{
  return m_z;
}

void geo_pos_conv::set_plane(double lat, double lon)
{
  m_PLato = lat;
  m_PLo = lon;
}

void geo_pos_conv::set_plane(int num)
{
  int lon_deg, lon_min, lat_deg, lat_min;  // longitude and latitude of origin of each plane in Japan
  if (num == 0)
  {
    lon_deg = 0;
    lon_min = 0;
    lat_deg = 0;
    lat_min = 0;
  }
  else if (num == 1)
  {
    lon_deg = 33;
    lon_min = 0;
    lat_deg = 129;
    lat_min = 30;
  }
  else if (num == 2)
  {
    lon_deg = 33;
    lon_min = 0;
    lat_deg = 131;
    lat_min = 0;
  }
  else if (num == 3)
  {
    lon_deg = 36;
    lon_min = 0;
    lat_deg = 132;
    lat_min = 10;
  }
  else if (num == 4)
  {
    lon_deg = 33;
    lon_min = 0;
    lat_deg = 133;
    lat_min = 30;
  }
  else if (num == 5)
  {
    lon_deg = 36;
    lon_min = 0;
    lat_deg = 134;
    lat_min = 20;
  }
  else if (num == 6)
  {
    lon_deg = 36;
    lon_min = 0;
    lat_deg = 136;
    lat_min = 0;
  }
  else if (num == 7)
  {
    // Changed by wang-ruifan
    /*lon_deg = 36;
    lon_min = 0;
    lat_deg = 137;
    lat_min = 10;*/

    lon_deg = 22;
    lon_min = 31;
    lat_deg = 113;
    lat_min = 56;
    // Changed end
  }
  else if (num == 8)
  {
    lon_deg = 36;
    lon_min = 0;
    lat_deg = 138;
    lat_min = 30;
  }
  else if (num == 9)
  {
    lon_deg = 36;
    lon_min = 0;
    lat_deg = 139;
    lat_min = 50;
  }
  else if (num == 10)
  {
    lon_deg = 40;
    lon_min = 0;
    lat_deg = 140;
    lat_min = 50;
  }
  else if (num == 11)
  {
    lon_deg = 44;
    lon_min = 0;
    lat_deg = 140;
    lat_min = 15;
  }
  else if (num == 12)
  {
    lon_deg = 44;
    lon_min = 0;
    lat_deg = 142;
    lat_min = 15;
  }
  else if (num == 13)
  {
    lon_deg = 44;
    lon_min = 0;
    lat_deg = 144;
    lat_min = 15;
  }
  else if (num == 14)
  {
    lon_deg = 26;
    lon_min = 0;
    lat_deg = 142;
    lat_min = 0;
  }
  else if (num == 15)
  {
    lon_deg = 26;
    lon_min = 0;
    lat_deg = 127;
    lat_min = 30;
  }
  else if (num == 16)
  {
    lon_deg = 26;
    lon_min = 0;
    lat_deg = 124;
    lat_min = 0;
  }
  else if (num == 17)
  {
    lon_deg = 26;
    lon_min = 0;
    lat_deg = 131;
    lat_min = 0;
  }
  else if (num == 18)
  {
    lon_deg = 20;
    lon_min = 0;
    lat_deg = 136;
    lat_min = 0;
  }
  else if (num == 19)
  {
    lon_deg = 26;
    lon_min = 0;
    lat_deg = 154;
    lat_min = 0;
  }

  // swap longitude and latitude
  m_PLo = M_PI * (static_cast<double>(lat_deg) + static_cast<double>(lat_min) / 60.0) / 180.0;
  m_PLato = M_PI * (static_cast<double>(lon_deg) + static_cast<double>(lon_min) / 60.0) / 180;
}

void geo_pos_conv::set_xyz(double cx, double cy, double cz)
{
  m_x = cx;
  m_y = cy;
  m_z = cz;
  conv_xyz2llh();
}

void geo_pos_conv::set_llh_nmea_degrees(double latd, double lond, double h)
{
  double lat, lad, lod, lon;
  // 1234.56 -> 12'34.56 -> 12+ 34.56/60

  if (latd > 0)
  {
    lad = floor(latd / 100.);
  }
  else
  {
    lad = ceil(latd / 100.);
  }
  lat = latd - lad * 100.;

  if (lond > 0)
  {
    lod = floor(lond / 100.);
  }
  else
  {
    lod = ceil(lond / 100.);
  }
  lon = lond - lod * 100.;

  // Changing Longitude and Latitude to Radians
  m_lat = (lad + lat / 60.0) * M_PI / 180;
  m_lon = (lod + lon / 60.0) * M_PI / 180;
  m_h = h;

  conv_llh2xyz();
}

void geo_pos_conv::llh_to_xyz(double lat, double lon, double ele)
{
  m_lat = lat * M_PI / 180;
  m_lon = lon * M_PI / 180;
  m_h = ele;

  conv_llh2xyz();
}

void geo_pos_conv::conv_llh2xyz(void)
{
  double PS;   //
  double PSo;  //
  double PDL;  //
  double Pt;   //
  double PN;   //
  double PW;   //

  double PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9;
  double PA, PB, PC, PD, PE, PF, PG, PH, PI;
  double Pe;   //
  double Pet;  //
  double Pnn;  //
  double AW, FW, Pmo;

  Pmo = 0.9999;

  /*WGS84 Parameters*/
  AW = 6378137.0;            // Semimajor Axis
  FW = 1.0 / 298.257222101;  // 298.257223563 //Geometrical flattening

  Pe = static_cast<double>(std::sqrt(2.0 * FW - std::pow(FW, 2)));
  Pet = static_cast<double>(std::sqrt(std::pow(Pe, 2) / (1.0 - std::pow(Pe, 2))));

  PA = static_cast<double>(1.0 + 3.0 / 4.0 * std::pow(Pe, 2) + 45.0 / 64.0 * std::pow(Pe, 4) +
    175.0 / 256.0 * std::pow(Pe, 6) + 11025.0 / 16384.0 * std::pow(Pe, 8) + 43659.0 / 65536.0 *
    std::pow(Pe, 10) + 693693.0 / 1048576.0 * std::pow(Pe, 12) + 19324305.0 / 29360128.0 * std::pow(Pe, 14) +
    4927697775.0 / 7516192768.0 * std::pow(Pe, 16));

  PB = static_cast<double>(3.0 / 4.0 * std::pow(Pe, 2) + 15.0 / 16.0 * std::pow(Pe, 4) + 525.0 / 512.0 *
    std::pow(Pe, 6) + 2205.0 / 2048.0 * std::pow(Pe, 8) + 72765.0 / 65536.0 * std::pow(Pe, 10) + 297297.0 / 262144.0 *
    std::pow(Pe, 12) + 135270135.0 / 117440512.0 * std::pow(Pe, 14) + 547521975.0 / 469762048.0 * std::pow(Pe, 16));

  PC = static_cast<double>(15.0 / 64.0 * std::pow(Pe, 4) + 105.0 / 256.0 * std::pow(Pe, 6) + 2205.0 / 4096.0 *
    std::pow(Pe, 8) + 10395.0 / 16384.0 * std::pow(Pe, 10) + 1486485.0 / 2097152.0 * std::pow(Pe, 12) +
    45090045.0 / 58720256.0 * std::pow(Pe, 14) + 766530765.0 / 939524096.0 * std::pow(Pe, 16));

  PD = static_cast<double>(35.0 / 512.0 * std::pow(Pe, 6) + 315.0 / 2048.0 * std::pow(Pe, 8) + 31185.0 / 131072.0 *
    std::pow(Pe, 10) + 165165.0 / 524288.0 * std::pow(Pe, 12) + 45090045.0 / 117440512.0 * std::pow(Pe, 14) +
    209053845.0 / 469762048.0 * std::pow(Pe, 16));

  PE = static_cast<double>(315.0 / 16384.0 * std::pow(Pe, 8) + 3465.0 / 65536.0 * std::pow(Pe, 10) +
    99099.0 / 1048576.0 * std::pow(Pe, 12) + 4099095.0 / 29360128.0 * std::pow(Pe, 14) + 348423075.0 / 1879048192.0 *
    std::pow(Pe, 16));

  PF = static_cast<double>(693.0 / 131072.0 * std::pow(Pe, 10) + 9009.0 / 524288.0 * std::pow(Pe, 12) +
    4099095.0 / 117440512.0 * std::pow(Pe, 14) + 26801775.0 / 469762048.0 * std::pow(Pe, 16));

  PG = static_cast<double>(3003.0 / 2097152.0 * std::pow(Pe, 12) + 315315.0 / 58720256.0 * std::pow(Pe, 14) +
    11486475.0 / 939524096.0 * std::pow(Pe, 16));

  PH = static_cast<double>(45045.0 / 117440512.0 * std::pow(Pe, 14) + 765765.0 / 469762048.0 * std::pow(Pe, 16));

  PI = static_cast<double>(765765.0 / 7516192768.0 * std::pow(Pe, 16));

  PB1 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PA;
  PB2 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PB / -2.0;
  PB3 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PC / 4.0;
  PB4 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PD / -6.0;
  PB5 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PE / 8.0;
  PB6 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PF / -10.0;
  PB7 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PG / 12.0;
  PB8 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PH / -14.0;
  PB9 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PI / 16.0;

  PS = static_cast<double>(PB1) * m_lat + PB2 * std::sin(2.0 * m_lat) + PB3 * std::sin(4.0 * m_lat) + PB4 *
    std::sin(6.0 * m_lat) + PB5 * std::sin(8.0 * m_lat) + PB6 * std::sin(10.0 * m_lat) + PB7 * std::sin(12.0 * m_lat) +
    PB8 * std::sin(14.0 * m_lat) + PB9 * std::sin(16.0 * m_lat);

  PSo = static_cast<double>(PB1) * m_PLato + PB2 * std::sin(2.0 * m_PLato) + PB3 * std::sin(4.0 * m_PLato) + PB4 *
    std::sin(6.0 * m_PLato) + PB5 * std::sin(8.0 * m_PLato) + PB6 * std::sin(10.0 * m_PLato) + PB7 *
    std::sin(12.0 * m_PLato) + PB8 * std::sin(14.0 * m_PLato) + PB9 * std::sin(16.0 * m_PLato);

  PDL = static_cast<double>(m_lon) - m_PLo;
  Pt = static_cast<double>(std::tan(m_lat));
  PW = static_cast<double>(std::sqrt(1.0 - std::pow(Pe, 2) * std::pow(std::sin(m_lat), 2)));
  PN = static_cast<double>(AW) / PW;
  Pnn = static_cast<double>(std::sqrt(std::pow(Pet, 2) * std::pow(std::cos(m_lat), 2)));

  m_x = static_cast<double>(((PS - PSo) + (1.0 / 2.0) * PN * std::pow(std::cos(m_lat), 2.0) * Pt * std::pow(PDL, 2.0) +
    (1.0 / 24.0) * PN * std::pow(std::cos(m_lat), 4) * Pt *
    (5.0 - std::pow(Pt, 2) + 9.0 * std::pow(Pnn, 2) + 4.0 * std::pow(Pnn, 4)) * std::pow(PDL, 4) -
    (1.0 / 720.0) * PN * std::pow(std::cos(m_lat), 6) * Pt *
    (-61.0 + 58.0 * std::pow(Pt, 2) - std::pow(Pt, 4) - 270.0 * std::pow(Pnn, 2) + 330.0 *
      std::pow(Pt, 2) * std::pow(Pnn, 2)) *
    std::pow(PDL, 6) - (1.0 / 40320.0) * PN * std::pow(std::cos(m_lat), 8) * Pt *
    (-1385.0 + 3111 * std::pow(Pt, 2) - 543 * std::pow(Pt, 4) + std::pow(Pt, 6)) * std::pow(PDL, 8)) * Pmo);

  m_y = static_cast<double>((PN * std::cos(m_lat) * PDL -
    1.0 / 6.0 * PN * std::pow(std::cos(m_lat), 3) * (-1 + std::pow(Pt, 2) - std::pow(Pnn, 2)) * std::pow(PDL, 3) -
    1.0 / 120.0 * PN * std::pow(std::cos(m_lat), 5) *
    (-5.0 + 18.0 * std::pow(Pt, 2) - std::pow(Pt, 4) - 14.0 * std::pow(Pnn, 2) + 58.0 * std::pow(Pt, 2) *
      std::pow(Pnn, 2)) * std::pow(PDL, 5) -
    1.0 / 5040.0 * PN * std::pow(std::cos(m_lat), 7) *
    (-61.0 + 479.0 * std::pow(Pt, 2) - 179.0 * std::pow(Pt, 4) + std::pow(Pt, 6)) * std::pow(PDL, 7)) * Pmo);

  m_z = m_h;

  Eigen::Matrix3f R;
  Eigen::RowVector3f T;
  Eigen::RowVector3f m_3;
  Eigen::RowVector3f m_4;
  
  // Added by wang-ruifan
  // 原始笛卡尔坐标（ m_y, m_x, m_z）分配给m_3向量，赋值顺序为y,x,z
  m_3 << m_y, m_x, m_z;

  // 旋转矩阵R是一个3x3矩阵，每行代表旋转后变换轴的方向
  R <<  -0.56697781, -0.81293222, 0.13295626,    
        0.8236486, -0.56179751, 0.0773727,
        0.01179574, 0.15337785, 0.98809721;
  // 平移动向量T表示旋转后应用于坐标的位移
  T << 1183.98240109, 801.45776715, -231.84002211;     

  // 变换后的坐标m_4是通过将原始坐标m_3乘以喜欢转矩阵R.transpose()的转置，然后加上平移向量得到的T
  m_4 = m_3 * R.transpose() + T;

  // 将转换后的坐标m_4分配回m_x，m_y,m_z变量，赋值顺序为y,x,z
  m_y = m_4[0];
  m_x = m_4[1];
  m_z = m_4[2];
  // Added end
}

void geo_pos_conv::conv_xyz2llh(void)
{
  // n/a
}
