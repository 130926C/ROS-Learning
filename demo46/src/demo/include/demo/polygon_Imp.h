#ifndef __POLYGON_IMP_H__
#define __POLYGON_IMP_H__

#include "polygon_base.h"

namespace polygon_base_ns
{
    class Triangle: public polygon_base_ns::PolygonBase{
        private:
            double side_length;
        public:
            Triangle(): side_length(0.0) {};
            double getLength(){ return 3.0 * this->side_length; }
            void init(double side_length) { this->side_length = side_length;}
    };

    class Square: public polygon_base_ns::PolygonBase{
        private:
            double side_length;
        public:
            Square(): side_length(0.0) {};
            double getLength() {return 4.0 * this->side_length; }
            void init(double side_length) {this->side_length = side_length; }
    };
} // namespace polygon_base_ns

#endif 