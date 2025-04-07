#ifndef COVERAGE_QUAD
#define COVERAGE_QUAD

struct CoverageQuad
{
        uint32_t sensor;
        uint32_t area;
        uint32_t originId;
        double dataScaled;

        inline bool operator==( const CoverageQuad& rhs ) const { return this->sensor == rhs.sensor && this->area == rhs.area && this->originId == rhs.originId && this->dataScaled == rhs.dataScaled; }
};

struct CoverageQuadLess
{
        bool operator( )( const CoverageQuad& lhs, const CoverageQuad& rhs ) const
        {
            // Compare sensor
            if ( lhs.sensor < rhs.sensor )
                return true;
            if ( lhs.sensor > rhs.sensor )
                return false;

            // Compare area
            if ( lhs.area < rhs.area )
                return true;
            if ( lhs.area > rhs.area )
                return false;

            // TODO: Now sensor & area are the same => compare dataScaled
            // "Less" means we prefer items with smaller cost
            return false;
        }
};

#endif // COVERAGE_QUAD