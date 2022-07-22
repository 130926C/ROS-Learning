#ifndef __POLYGON_BASE_H__
#define __POLYGON_BASE_H__

namespace polygon_base_ns
{
    /*
        pluginlib 要求在基类中保留一个无参构造
    */
    class PolygonBase{
        protected:
            PolygonBase() {};
        public:
            virtual double getLength() = 0;
            virtual void init(double side_length) = 0;
    };
} // namespace polygon_base_ns

#endif 