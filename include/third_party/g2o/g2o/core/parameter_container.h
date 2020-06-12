//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_PARAMETER_CONTAINER_H
#define ORB_SLAM2_KINETIC_PARAMETER_CONTAINER_H

#include <iosfwd>
#include <map>
#include <string>

namespace g2o {

    class Parameter;

    /**
     * \brief map id to parameters
     */
    class ParameterContainer : protected std::map<int, Parameter*>
    {
    public:
        typedef std::map<int, Parameter*> BaseClass;

        /**
         * create a container for the parameters.
         * @param isMainStorage_ pointers to the parameters are owned by this container, i.e., freed in its constructor
         */
        ParameterContainer(bool isMainStorage_=true);
        virtual ~ParameterContainer();
        //! add parameter to the container
        bool addParameter(Parameter* p);
        //! return a parameter based on its ID
        Parameter* getParameter(int id);
        //! remove a parameter from the container, i.e., the user now owns the pointer
        Parameter* detachParameter(int id);
        //! read parameters from a stream
        virtual bool read(std::istream& is, const std::map<std::string, std::string>* renamedMap =0);
        //! write the data to a stream
        virtual bool write(std::ostream& os) const;
        bool isMainStorage() const {return _isMainStorage;}
        void clear();

        // stuff of the base class that should re-appear
        using BaseClass::size;

    protected:
        bool _isMainStorage;
    };

} // end namespace

#endif //ORB_SLAM2_KINETIC_PARAMETER_CONTAINER_H
