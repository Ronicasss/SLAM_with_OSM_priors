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

#ifndef EDGE_SE2_POINTXY_CUSTOM_HPP
#define EDGE_SE2_POINTXY_CUSTOM_HPP

#include <Eigen/Dense>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/types/slam2d_addons/types_slam2d_addons.h>

namespace g2o {
  class EdgeSE2PointXYCustom : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSE2, g2o::VertexPointXY>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      EdgeSE2PointXYCustom() : g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSE2, g2o::VertexPointXY>() {
        _information.setIdentity();
        _error.setZero();
      }

      void computeError() override
      {
        const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
        const VertexPointXY* l2 = static_cast<const VertexPointXY*>(_vertices[1]);
        //std::cout << "io2 " << (static_cast<double>(graph()->edges().size())) << std::endl;
       
        /*auto edge_itr = l2->edges().begin();
        int max = 0;
      
        for(int i = 0; edge_itr != l2->edges().end(); edge_itr++, i++) {
         
          g2o::HyperGraph::Edge* edge = *edge_itr;
   
          g2o::EdgeSE2PointXYCustom* edge_se2_xy = dynamic_cast<g2o::EdgeSE2PointXYCustom*>(edge);
          if(i==0 || edge_se2_xy->id() > max)
            max = edge_se2_xy->id();
        }
        //std::cout << "max: " << max << std::endl;
        //std::cout << "edge id: " << _id << std::endl;

        double weight_t = (_id)/(static_cast<double>(max));
        double weight = 0.0;
        if(weight_t < 0.5)
          weight = (exp(weight_t)-1)/(exp(1)-1);
        else
          weight = weight_t;*/
        //std::cout << "weight_t: " << weight_t << std::endl;
        //std::cout << "weight: " << weight << std::endl;
        _error = ((v1->estimate().inverse() * l2->estimate()) - _measurement);
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
} // end namespace

#endif