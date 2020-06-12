//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_PARAMETER_H
#define ORB_SLAM2_KINETIC_PARAMETER_H

#include <iosfwd>

#include "hyper_graph.h"

namespace g2o {

    class  Parameter : public HyperGraph::HyperGraphElement
    {
    public:
        Parameter();
        virtual ~Parameter() {};
        //! read the data from a stream
        virtual bool read(std::istream& is) = 0;
        //! write the data to a stream
        virtual bool write(std::ostream& os) const = 0;
        int id() const {return _id;}
        void setId(int id_);
        virtual HyperGraph::HyperGraphElementType elementType() const { return HyperGraph::HGET_PARAMETER;}
    protected:
        int _id;
    };

    typedef std::vector<Parameter*> ParameterVector;

} // end namespace

#endif //ORB_SLAM2_KINETIC_PARAMETER_H
