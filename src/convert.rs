use bevy::prelude::Component;
use bevy_rapier3d::geometry::ActiveHooks;
use bevy_rapier3d::prelude::{ActiveCollisionTypes, ActiveEvents, CoefficientCombineRule, Group};
use bevy_rapier3d::rapier;

pub trait IntoBevy {
    type Target;

    fn into_bevy(self) -> Self::Target;
}

impl IntoBevy for rapier::prelude::CoefficientCombineRule {
    type Target = CoefficientCombineRule;
    fn into_bevy(self) -> Self::Target {
        match self {
            rapier::prelude::CoefficientCombineRule::Average =>
                CoefficientCombineRule::Average,
            rapier::prelude::CoefficientCombineRule::Min =>
                CoefficientCombineRule::Min,
            rapier::prelude::CoefficientCombineRule::Max =>
                CoefficientCombineRule::Max,
            rapier::prelude::CoefficientCombineRule::Multiply =>
                CoefficientCombineRule::Multiply,
        }
    }
}

impl IntoBevy for rapier::prelude::ActiveCollisionTypes {
    type Target = ActiveCollisionTypes;
    fn into_bevy(self) -> Self::Target {
        Self::Target::from_bits_retain(self.bits())
    }
}

impl IntoBevy for rapier::prelude::Group {
    type Target = Group;
    fn into_bevy(self) -> Self::Target {
        Self::Target::from_bits_retain(self.bits())
    }
}

impl IntoBevy for rapier::prelude::ActiveHooks {
    type Target = ActiveHooks;
    fn into_bevy(self) -> Self::Target {
        Self::Target::from_bits_retain(self.bits())
    }
}

impl IntoBevy for rapier::prelude::ActiveEvents {
    type Target = ActiveEvents;
    fn into_bevy(self) -> Self::Target {
        Self::Target::from_bits_retain(self.bits())
    }
}
