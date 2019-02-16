//#line 2 "/opt/ros/kinetic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"
// *********************************************************
//
// File autogenerated for the move_base package
// by the dynamic_reconfigure package.
// Please do not edit.
//
// ********************************************************/

#ifndef __move_base__MOVEBASECONFIG_H__
#define __move_base__MOVEBASECONFIG_H__

#if __cplusplus >= 201103L
#define DYNAMIC_RECONFIGURE_FINAL final
#else
#define DYNAMIC_RECONFIGURE_FINAL
#endif

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/config_init_mutex.h>
#include <boost/any.hpp>

namespace move_base
{
  class MoveBaseConfigStatics;

  class MoveBaseConfig
  {
  public:
    class AbstractParamDescription : public dynamic_reconfigure::ParamDescription
    {
    public:
      AbstractParamDescription(std::string n, std::string t, uint32_t l,
          std::string d, std::string e)
      {
        name = n;
        type = t;
        level = l;
        description = d;
        edit_method = e;
      }

      virtual void clamp(MoveBaseConfig &config, const MoveBaseConfig &max, const MoveBaseConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const MoveBaseConfig &config1, const MoveBaseConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, MoveBaseConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const MoveBaseConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, MoveBaseConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const MoveBaseConfig &config) const = 0;
      virtual void getValue(const MoveBaseConfig &config, boost::any &val) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;

    // Final keyword added to class because it has virtual methods and inherits
    // from a class with a non-virtual destructor.
    template <class T>
    class ParamDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string a_name, std::string a_type, uint32_t a_level,
          std::string a_description, std::string a_edit_method, T MoveBaseConfig::* a_f) :
        AbstractParamDescription(a_name, a_type, a_level, a_description, a_edit_method),
        field(a_f)
      {}

      T (MoveBaseConfig::* field);

      virtual void clamp(MoveBaseConfig &config, const MoveBaseConfig &max, const MoveBaseConfig &min) const
      {
        if (config.*field > max.*field)
          config.*field = max.*field;

        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const MoveBaseConfig &config1, const MoveBaseConfig &config2) const
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, MoveBaseConfig &config) const
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const MoveBaseConfig &config) const
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, MoveBaseConfig &config) const
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const MoveBaseConfig &config) const
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }

      virtual void getValue(const MoveBaseConfig &config, boost::any &val) const
      {
        val = config.*field;
      }
    };

    class AbstractGroupDescription : public dynamic_reconfigure::Group
    {
      public:
      AbstractGroupDescription(std::string n, std::string t, int p, int i, bool s)
      {
        name = n;
        type = t;
        parent = p;
        state = s;
        id = i;
      }

      std::vector<AbstractParamDescriptionConstPtr> abstract_parameters;
      bool state;

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &config) const =0;
      virtual void updateParams(boost::any &cfg, MoveBaseConfig &top) const= 0;
      virtual void setInitialState(boost::any &cfg) const = 0;


      void convertParams()
      {
        for(std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = abstract_parameters.begin(); i != abstract_parameters.end(); ++i)
        {
          parameters.push_back(dynamic_reconfigure::ParamDescription(**i));
        }
      }
    };

    typedef boost::shared_ptr<AbstractGroupDescription> AbstractGroupDescriptionPtr;
    typedef boost::shared_ptr<const AbstractGroupDescription> AbstractGroupDescriptionConstPtr;

    // Final keyword added to class because it has virtual methods and inherits
    // from a class with a non-virtual destructor.
    template<class T, class PT>
    class GroupDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractGroupDescription
    {
    public:
      GroupDescription(std::string a_name, std::string a_type, int a_parent, int a_id, bool a_s, T PT::* a_f) : AbstractGroupDescription(a_name, a_type, a_parent, a_id, a_s), field(a_f)
      {
      }

      GroupDescription(const GroupDescription<T, PT>& g): AbstractGroupDescription(g.name, g.type, g.parent, g.id, g.state), field(g.field), groups(g.groups)
      {
        parameters = g.parameters;
        abstract_parameters = g.abstract_parameters;
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        if(!dynamic_reconfigure::ConfigTools::getGroupState(msg, name, (*config).*field))
          return false;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          if(!(*i)->fromMessage(msg, n))
            return false;
        }

        return true;
      }

      virtual void setInitialState(boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        T* group = &((*config).*field);
        group->state = state;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = boost::any(&((*config).*field));
          (*i)->setInitialState(n);
        }

      }

      virtual void updateParams(boost::any &cfg, MoveBaseConfig &top) const
      {
        PT* config = boost::any_cast<PT*>(cfg);

        T* f = &((*config).*field);
        f->setParams(top, abstract_parameters);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          (*i)->updateParams(n, top);
        }
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &cfg) const
      {
        const PT config = boost::any_cast<PT>(cfg);
        dynamic_reconfigure::ConfigTools::appendGroup<T>(msg, name, id, parent, config.*field);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          (*i)->toMessage(msg, config.*field);
        }
      }

      T (PT::* field);
      std::vector<MoveBaseConfig::AbstractGroupDescriptionConstPtr> groups;
    };

class DEFAULT
{
  public:
    DEFAULT()
    {
      state = true;
      name = "Default";
    }

    void setParams(MoveBaseConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
      {
        boost::any val;
        (*_i)->getValue(config, val);

        if("base_global_planner"==(*_i)->name){base_global_planner = boost::any_cast<std::string>(val);}
        if("base_local_planner"==(*_i)->name){base_local_planner = boost::any_cast<std::string>(val);}
        if("planner_frequency"==(*_i)->name){planner_frequency = boost::any_cast<double>(val);}
        if("controller_frequency"==(*_i)->name){controller_frequency = boost::any_cast<double>(val);}
        if("planner_patience"==(*_i)->name){planner_patience = boost::any_cast<double>(val);}
        if("controller_patience"==(*_i)->name){controller_patience = boost::any_cast<double>(val);}
        if("conservative_reset_dist"==(*_i)->name){conservative_reset_dist = boost::any_cast<double>(val);}
        if("recovery_behavior_enabled"==(*_i)->name){recovery_behavior_enabled = boost::any_cast<bool>(val);}
        if("clearing_rotation_allowed"==(*_i)->name){clearing_rotation_allowed = boost::any_cast<bool>(val);}
        if("shutdown_costmaps"==(*_i)->name){shutdown_costmaps = boost::any_cast<bool>(val);}
        if("oscillation_timeout"==(*_i)->name){oscillation_timeout = boost::any_cast<double>(val);}
        if("oscillation_distance"==(*_i)->name){oscillation_distance = boost::any_cast<double>(val);}
        if("restore_defaults"==(*_i)->name){restore_defaults = boost::any_cast<bool>(val);}
      }
    }

    std::string base_global_planner;
std::string base_local_planner;
double planner_frequency;
double controller_frequency;
double planner_patience;
double controller_patience;
double conservative_reset_dist;
bool recovery_behavior_enabled;
bool clearing_rotation_allowed;
bool shutdown_costmaps;
double oscillation_timeout;
double oscillation_distance;
bool restore_defaults;

    bool state;
    std::string name;

    
}groups;



//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      std::string base_global_planner;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      std::string base_local_planner;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double planner_frequency;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double controller_frequency;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double planner_patience;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double controller_patience;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double conservative_reset_dist;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      bool recovery_behavior_enabled;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      bool clearing_rotation_allowed;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      bool shutdown_costmaps;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double oscillation_timeout;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double oscillation_distance;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      bool restore_defaults;
//#line 228 "/opt/ros/kinetic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();

      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        if ((*i)->fromMessage(msg, *this))
          count++;

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i ++)
      {
        if ((*i)->id == 0)
        {
          boost::any n = boost::any(this);
          (*i)->updateParams(n, *this);
          (*i)->fromMessage(msg, n);
        }
      }

      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("MoveBaseConfig::__fromMessage__ called with an unexpected parameter.");
        ROS_ERROR("Booleans:");
        for (unsigned int i = 0; i < msg.bools.size(); i++)
          ROS_ERROR("  %s", msg.bools[i].name.c_str());
        ROS_ERROR("Integers:");
        for (unsigned int i = 0; i < msg.ints.size(); i++)
          ROS_ERROR("  %s", msg.ints[i].name.c_str());
        ROS_ERROR("Doubles:");
        for (unsigned int i = 0; i < msg.doubles.size(); i++)
          ROS_ERROR("  %s", msg.doubles[i].name.c_str());
        ROS_ERROR("Strings:");
        for (unsigned int i = 0; i < msg.strs.size(); i++)
          ROS_ERROR("  %s", msg.strs[i].name.c_str());
        // @todo Check that there are no duplicates. Make this error more
        // explicit.
        return false;
      }
      return true;
    }

    // This version of __toMessage__ is used during initialization of
    // statics when __getParamDescriptions__ can't be called yet.
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__, const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toMessage(msg, *this);

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        if((*i)->id == 0)
        {
          (*i)->toMessage(msg, *this);
        }
      }
    }

    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      __toMessage__(msg, __param_descriptions__, __group_descriptions__);
    }

    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      static bool setup=false;

      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->fromServer(nh, *this);

      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++){
        if (!setup && (*i)->id == 0) {
          setup = true;
          boost::any n = boost::any(this);
          (*i)->setInitialState(n);
        }
      }
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const MoveBaseConfig &__max__ = __getMax__();
      const MoveBaseConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const MoveBaseConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->calcLevel(level, config, *this);
      return level;
    }

    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const MoveBaseConfig &__getDefault__();
    static const MoveBaseConfig &__getMax__();
    static const MoveBaseConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    static const std::vector<AbstractGroupDescriptionConstPtr> &__getGroupDescriptions__();

  private:
    static const MoveBaseConfigStatics *__get_statics__();
  };

  template <> // Max and min are ignored for strings.
  inline void MoveBaseConfig::ParamDescription<std::string>::clamp(MoveBaseConfig &config, const MoveBaseConfig &max, const MoveBaseConfig &min) const
  {
    (void) config;
    (void) min;
    (void) max;
    return;
  }

  class MoveBaseConfigStatics
  {
    friend class MoveBaseConfig;

    MoveBaseConfigStatics()
    {
MoveBaseConfig::GroupDescription<MoveBaseConfig::DEFAULT, MoveBaseConfig> Default("Default", "", 0, 0, true, &MoveBaseConfig::groups);
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.base_global_planner = "";
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.base_global_planner = "";
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.base_global_planner = "navfn/NavfnROS";
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<std::string>("base_global_planner", "str", 0, "The name of the plugin for the global planner to use with move_base.", "", &MoveBaseConfig::base_global_planner)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<std::string>("base_global_planner", "str", 0, "The name of the plugin for the global planner to use with move_base.", "", &MoveBaseConfig::base_global_planner)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.base_local_planner = "";
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.base_local_planner = "";
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.base_local_planner = "base_local_planner/TrajectoryPlannerROS";
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<std::string>("base_local_planner", "str", 0, "The name of the plugin for the local planner to use with move_base.", "", &MoveBaseConfig::base_local_planner)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<std::string>("base_local_planner", "str", 0, "The name of the plugin for the local planner to use with move_base.", "", &MoveBaseConfig::base_local_planner)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.planner_frequency = 0.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.planner_frequency = 100.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.planner_frequency = 0.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<double>("planner_frequency", "double", 0, "The rate in Hz at which to run the planning loop.", "", &MoveBaseConfig::planner_frequency)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<double>("planner_frequency", "double", 0, "The rate in Hz at which to run the planning loop.", "", &MoveBaseConfig::planner_frequency)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.controller_frequency = 0.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.controller_frequency = 100.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.controller_frequency = 20.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<double>("controller_frequency", "double", 0, "The rate in Hz at which to run the control loop and send velocity commands to the base.", "", &MoveBaseConfig::controller_frequency)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<double>("controller_frequency", "double", 0, "The rate in Hz at which to run the control loop and send velocity commands to the base.", "", &MoveBaseConfig::controller_frequency)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.planner_patience = 0.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.planner_patience = 100.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.planner_patience = 5.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<double>("planner_patience", "double", 0, "How long the planner will wait in seconds in an attempt to find a valid plan before space-clearing operations are performed.", "", &MoveBaseConfig::planner_patience)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<double>("planner_patience", "double", 0, "How long the planner will wait in seconds in an attempt to find a valid plan before space-clearing operations are performed.", "", &MoveBaseConfig::planner_patience)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.controller_patience = 0.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.controller_patience = 100.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.controller_patience = 5.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<double>("controller_patience", "double", 0, "How long the controller will wait in seconds without receiving a valid control before space-clearing operations are performed.", "", &MoveBaseConfig::controller_patience)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<double>("controller_patience", "double", 0, "How long the controller will wait in seconds without receiving a valid control before space-clearing operations are performed.", "", &MoveBaseConfig::controller_patience)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.conservative_reset_dist = 0.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.conservative_reset_dist = 50.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.conservative_reset_dist = 3.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<double>("conservative_reset_dist", "double", 0, "The distance away from the robot in meters at which obstacles will be cleared from the costmap when attempting to clear space in the map.", "", &MoveBaseConfig::conservative_reset_dist)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<double>("conservative_reset_dist", "double", 0, "The distance away from the robot in meters at which obstacles will be cleared from the costmap when attempting to clear space in the map.", "", &MoveBaseConfig::conservative_reset_dist)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.recovery_behavior_enabled = 0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.recovery_behavior_enabled = 1;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.recovery_behavior_enabled = 1;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<bool>("recovery_behavior_enabled", "bool", 0, "Whether or not to enable the move_base recovery behaviors to attempt to clear out space.", "", &MoveBaseConfig::recovery_behavior_enabled)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<bool>("recovery_behavior_enabled", "bool", 0, "Whether or not to enable the move_base recovery behaviors to attempt to clear out space.", "", &MoveBaseConfig::recovery_behavior_enabled)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.clearing_rotation_allowed = 0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.clearing_rotation_allowed = 1;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.clearing_rotation_allowed = 1;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<bool>("clearing_rotation_allowed", "bool", 0, "Determines whether or not the robot will attempt an in-place rotation when attempting to clear out space.", "", &MoveBaseConfig::clearing_rotation_allowed)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<bool>("clearing_rotation_allowed", "bool", 0, "Determines whether or not the robot will attempt an in-place rotation when attempting to clear out space.", "", &MoveBaseConfig::clearing_rotation_allowed)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.shutdown_costmaps = 0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.shutdown_costmaps = 1;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.shutdown_costmaps = 0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<bool>("shutdown_costmaps", "bool", 0, "Determines whether or not to shutdown the costmaps of the node when move_base is in an inactive state", "", &MoveBaseConfig::shutdown_costmaps)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<bool>("shutdown_costmaps", "bool", 0, "Determines whether or not to shutdown the costmaps of the node when move_base is in an inactive state", "", &MoveBaseConfig::shutdown_costmaps)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.oscillation_timeout = 0.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.oscillation_timeout = 60.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.oscillation_timeout = 0.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<double>("oscillation_timeout", "double", 0, "How long in seconds to allow for oscillation before executing recovery behaviors.", "", &MoveBaseConfig::oscillation_timeout)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<double>("oscillation_timeout", "double", 0, "How long in seconds to allow for oscillation before executing recovery behaviors.", "", &MoveBaseConfig::oscillation_timeout)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.oscillation_distance = 0.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.oscillation_distance = 10.0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.oscillation_distance = 0.5;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<double>("oscillation_distance", "double", 0, "How far in meters the robot must move to be considered not to be oscillating.", "", &MoveBaseConfig::oscillation_distance)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<double>("oscillation_distance", "double", 0, "How far in meters the robot must move to be considered not to be oscillating.", "", &MoveBaseConfig::oscillation_distance)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.restore_defaults = 0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.restore_defaults = 1;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.restore_defaults = 0;
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<bool>("restore_defaults", "bool", 0, "Restore to the original configuration", "", &MoveBaseConfig::restore_defaults)));
//#line 290 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(MoveBaseConfig::AbstractParamDescriptionConstPtr(new MoveBaseConfig::ParamDescription<bool>("restore_defaults", "bool", 0, "Restore to the original configuration", "", &MoveBaseConfig::restore_defaults)));
//#line 245 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.convertParams();
//#line 245 "/opt/ros/kinetic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __group_descriptions__.push_back(MoveBaseConfig::AbstractGroupDescriptionConstPtr(new MoveBaseConfig::GroupDescription<MoveBaseConfig::DEFAULT, MoveBaseConfig>(Default)));
//#line 366 "/opt/ros/kinetic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

      for (std::vector<MoveBaseConfig::AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        __description_message__.groups.push_back(**i);
      }
      __max__.__toMessage__(__description_message__.max, __param_descriptions__, __group_descriptions__);
      __min__.__toMessage__(__description_message__.min, __param_descriptions__, __group_descriptions__);
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__, __group_descriptions__);
    }
    std::vector<MoveBaseConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    std::vector<MoveBaseConfig::AbstractGroupDescriptionConstPtr> __group_descriptions__;
    MoveBaseConfig __max__;
    MoveBaseConfig __min__;
    MoveBaseConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;

    static const MoveBaseConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static MoveBaseConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &MoveBaseConfig::__getDescriptionMessage__()
  {
    return __get_statics__()->__description_message__;
  }

  inline const MoveBaseConfig &MoveBaseConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }

  inline const MoveBaseConfig &MoveBaseConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }

  inline const MoveBaseConfig &MoveBaseConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }

  inline const std::vector<MoveBaseConfig::AbstractParamDescriptionConstPtr> &MoveBaseConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const std::vector<MoveBaseConfig::AbstractGroupDescriptionConstPtr> &MoveBaseConfig::__getGroupDescriptions__()
  {
    return __get_statics__()->__group_descriptions__;
  }

  inline const MoveBaseConfigStatics *MoveBaseConfig::__get_statics__()
  {
    const static MoveBaseConfigStatics *statics;

    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = MoveBaseConfigStatics::get_instance();

    return statics;
  }


}

#undef DYNAMIC_RECONFIGURE_FINAL

#endif // __MOVEBASERECONFIGURATOR_H__
