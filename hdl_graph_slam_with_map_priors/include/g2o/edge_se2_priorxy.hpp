// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef EDGE_SE2_PRIORXY_HPP
#define EDGE_SE2_PRIORXY_HPP

#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/types/slam2d_addons/types_slam2d_addons.h>

namespace g2o {
  /**
   * @brief this class links a pointxy prior to a se2 vertex
   */
class EdgeSE2PriorXY : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE2> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE2PriorXY() : g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE2>() {}

  void computeError() override {
    const g2o::VertexSE2* v1 = static_cast<const g2o::VertexSE2*>(_vertices[0]);

    Eigen::Vector2d estimate = v1->estimate().translation();
    _error = estimate - _measurement;
  }

  void setMeasurement(const Eigen::Vector2d& m) override {
    _measurement = m;
  }

  virtual bool read(std::istream& is) override {
    Eigen::Vector2d v;
    is >> v(0) >> v(1);
    setMeasurement(v);
    for(int i = 0; i < information().rows(); ++i)
      for(int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if(i != j) information()(j, i) = information()(i, j);
      }
    return true;
  }
  virtual bool write(std::ostream& os) const override {
    Eigen::Vector2d v = _measurement;
    os << v(0) << " " << v(1) << " ";
    for(int i = 0; i < information().rows(); ++i)
      for(int j = i; j < information().cols(); ++j) os << " " << information()(i, j);
    return os.good();
  }
};
}  // namespace g2o

#endif  // EDGE_SE3_PRIORXY_HPP
