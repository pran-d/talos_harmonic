#!/usr/bin/env python

"""Module definig Invoke launch utils."""

from collections.abc import (
    Callable,
    Iterable,
)
from typing import (
    Any,
    Dict,
    List,
    Optional,
    Protocol,
    Text,
    TypeAlias,
    TypeVar,
    Union,
    runtime_checkable,
)

from launch import (
    Action,
    LaunchContext,
    LaunchDescriptionEntity,
    Substitution,
)

T = TypeVar('T')
U = TypeVar('U')


# Define the FunctionSubstitution traits (callable on LaunchContext)
@runtime_checkable
class FunctionSubstitution(Protocol[T]):
    def __call__[T](self, context: LaunchContext) -> T: ...


SubstitutionOr: TypeAlias = Union[
    Substitution,
    FunctionSubstitution[T],
    T
]


def substitute(
        context: LaunchContext,
        obj: SubstitutionOr[T]
) -> Union[T, Text]:
    """Perform the substitution of the given value, if relevant.

    Note
    ----
    Dispatch call priority are as follows:
    1. Function-like API (returns T)
    2. launch.Substitution.perform() (returns only Text)
    3. Forward value the value
    """
    if isinstance(obj, FunctionSubstitution):
        return obj(context)
    elif isinstance(obj, Substitution):
        return obj.perform(context)
    else:
        return obj


class Invoke[T](Action, Substitution):
    """Invoke the given function with arguments evaluated when needed.

    This is meant to be used as replacement of OpaqueFunction as Action and can
    also be used as a Substitution for any ROS launch related operation.

    It also implements the FunctionSubstituion[T] traits, therefore calling
    substitute() on it returns __call__, and not perform.

    Any arguments provided that are **not Substituable** will be directly
    forwarded to the function.
    Substituable arguments (FunctionSubstituion[T] or launch.Substitution) will
    first be evaluated, given a LaunchContext, and then their values will be
    forwarded to the function call.

    You can obtain the raw Invoke result through the __call__(LaunchContext) ->
    T interface.

    When executing `.execute()` from the launch.Action API, this effectively
    calls __call__ but filter out the result T if it's not a list of
    LaunchDescriptionEntity or a single LaunchDescriptionEntity.

    When executing `.perform()` from the launch.Substituion API, this
    effectively calls __call__ but raises an error when the return type is not
    a Text, as expected by any launch substitution.

    Additionnally, Invoke provide a 'Chainable-like' interface (`.and_then()`
    and `.and_then_with_key()`), which enable users to call an other function,
    with the result of the previously function call, creating a function chain
    call.

    Examples
    --------
    - Invoke(foo, 'a', 1).and_then(bar, 'b', 3):
      -> bar(foo('a', 1), 'b', 3)
    - Invoke(foo, 1, toto='a').and_then(bar, 'b', titi=3):
      -> bar(foo(1, toto='a'), 'b', titi=3)
    - Invoke(foo).and_then_with_key('s', bar, 1):
      -> bar(1, s=foo())
    """

    def __init__(
            self,
            f: Callable[[...], T],
            *args: SubstitutionOr[Any],
            **kwargs: SubstitutionOr[Any],
    ) -> None:
        """Construct the Invoke action from the given function/args.

        Parameters
        ----------
        f: Callable[..., T]
          Fun called later on, with arguments evaluated from a LaunchContext
        args: SubstitutionOr[Any]
          Arguments forwarded to the function call, after evaluation
        kwargs: SubstitutionOr[Any]
          Keywords arguments forwarded to the function call, after evaluation
        """
        # NOTE: Ignore the Action kwargs ?
        super().__init__()

        self.__f = f
        self.__args = args
        self.__kwargs = kwargs

    def and_then(
            self,
            f: Callable[[...], U],
            *args: SubstitutionOr[Any],
            **kwargs: SubstitutionOr[Any],
    ) -> 'Invoke[U]':
        """Call a new function f with previous results forwared as 1rst arg.

        Parameters
        ----------
        f: Callable[..., T]
          Fun called later on, with arguments evaluated from a LaunchContext
        args: SubstitutionOr[Any]
          Arguments forwarded to the function call, after evaluation
        kwargs: SubstitutionOr[Any]
          Keywords arguments forwarded to the function call, after evaluation

        Returns
        -------
        Invoke[U]
          A new Invoke instance containing the previous Invoke call as function
          arguments
        """
        return Invoke(
            f,
            self,
            *args,
            **kwargs,
        )

    def and_then_with_key(
            self,
            key: Text,
            f: Callable[[...], U],
            *args: SubstitutionOr[Any],
            **kwargs: SubstitutionOr[Any],
    ) -> 'Invoke[U]':
        """Call a new function f with previous call result forwared with a key.

        Parameters
        ----------
        key: Text
          Key used to forward the previous result to in the next function call
        f: Callable[..., T]
          Fun called later on, with arguments evaluated from a LaunchContext
        args: SubstitutionOr[Any]
          Arguments forwarded to the function call, after evaluation
        kwargs: SubstitutionOr[Any]
          Keywords arguments forwarded to the function call, after evaluation

        Returns
        -------
        Invoke[U]
          A new Invoke instance containing the previous Invoke call as function
          arguments
        """
        kwargs[key] = self
        return Invoke[U](
            f,
            *args,
            **kwargs,
        )

    def describe_args(self, *, separator: Text = ', ') -> Text:
        """Create a repr string of the arguments only."""
        args_repr = separator.join(
            (
                repr(arg)
                for arg in self.__args
            )
        )

        args_repr += separator.join(
            (
                '{k}={v}'.format(k=k, v=repr(v))
                for k, v in self.__kwargs.items()
            )
        )

        return args_repr

    def describe(
            self,
            *other_args: Any,
            fmt: Text = '{f}({args})',
            **other_kwargs: Any
    ) -> Text:
        """Create a repr of the given invoke, using the provided fmt string.

        Parameters
        ----------
        fmt: Text
          Format string to use (default to '{f}({args})').
          The following named format identifier are used:
          - f: Correspond to the repr of the function;
          - args: Correspond to the output of describe_args()
        other_args: Any
          Arguments forwarded to describe_args()
        other_kwargs: Any
          Keyword arguments forwarded to describe_args()

        Returns
        -------
        Text:
          A describe of the Invoke action
        """
        return fmt.format(
            f=repr(self.__f),
            args=self.describe_args(*other_args, *other_kwargs),
        )

    def __repr__(self) -> Text:
        """Return describe()."""
        return self.describe()

    def __call__(self, context: LaunchContext) -> T:
        """Perform the postpone args evaluation and function call.

        Parameters
        ----------
        context: LaunchContext
          Launch context use to perform substitution, if relevant

        Returns
        -------
        T
          The result of calling the function, with arguments evaluated
        """
        return self.__f(
            *[substitute(context, arg) for arg in self.__args],
            **{k: substitute(context, v) for k, v in self.__kwargs.items()}
        )

    def perform(self, context: LaunchContext) -> Text:
        """Perform the Substitution given the LaunchContext.

        Note
        ----
        Same as calling call except that if the result is not a of type Text
        (expected by the Substitution API), it raises an exception.

        Parameters
        ----------
        context: LaunchContext
          Launch context use to perform substitution, if relevant

        Returns
        -------
        Text
          The evaluated value

        Raises
        ------
        RuntimeError
          When T != Text (return type of the function f)
        """
        value = self.__call__(context)

        if not isinstance(value, Text):
            raise RuntimeError(
                (
                    'Invoke returned a "{t}" instead of the expected Text '
                    ' when doing .perform() -> Text.'
                ).format(t=type(value))
            )

        return value

    def execute(
            self,
            context: LaunchContext
    ) -> Optional[List[LaunchDescriptionEntity]]:
        """Execute the action following launch.Action API.

        Note
        ----
        Same as calling __call__ except that the output is filtered to match
        the expected Action API

        Parameters
        ----------
        context: LaunchContext
          Launch context use to perform substitution, if relevant

        Returns
        -------
        Optional[List[LaunchDescriptionEntity]]
          The result of calling the action, with arguments evaluated
        """
        value = self.__call__(context)

        if isinstance(value, LaunchDescriptionEntity):
            return [value]
        elif isinstance(value, list) and all(
                isinstance(item, LaunchDescriptionEntity)
                for item in value
        ):
            return value
        else:
            return None


def evaluate_args(
        arg: SubstitutionOr[T],
        *others: SubstitutionOr[Any],
) -> Invoke[Union[T, List[Any]]]:
    """Create an Invoke instance that evaluate all args.

    Note
    ----
    If only one arg is provided, the result will be this arg
    evaluated. Otherwise, the result will be a List of evaluated args.

    Parameters
    ----------
    arg: SubstitutionOr[Any]
      A single arg to evaluate
    others: SubstitutionOr[Any]
      Any other arguments to evaluate

    Returns
    -------
    Invoke[Union[Any, List[Any]]]
      An Invoke instance that will return the arguments evaluated
    """
    return Invoke(
        lambda a, *o: a if len(o) == 0 else [a] + o,
        arg,
        *others,
    )


def evaluate_kwargs(
        **kwargs: SubstitutionOr[Any],
) -> Invoke[Dict[Text, Any]]:
    """Create an Invoke instance that evaluate the key's values.

    Parameters
    ----------
    kwargs: SubstitutionOr[Any]
      Keyword arguments with value to evaluate

    Returns
    -------
    Invoke[Dict[Text, Any]]
      An Invoke instance that will return the kwargs dict with values evaluated
    """
    return Invoke(
        lambda **kwargs: kwargs,
        **kwargs,
    )


def evaluate_list(
        iterable: Iterable[SubstitutionOr[Any]]
) -> Invoke[List[Any]]:
    """Create an Invoke instance that evaluate all values inside l.

    Parameters
    ----------
    iterable: Iterable[SubstitutionOr[T]]
      List of values that needs to be evaluated

    Returns
    -------
    Invoke[List[Any]]
      An Invoke instance that return a list of all values evaluated
    """
    return Invoke(lambda *args: args, *iterable)


def evaluate_dict(
        mapping: Dict[SubstitutionOr[Any], SubstitutionOr[Any]],
) -> Invoke[Dict[Any, Any]]:
    """Create an Invoke instance that evaluate keys/values of a dict.

    Parameters
    ----------
    mapping: Dict[SubstitutionOr[Any], SubstitutionOr[Any]]
      Arguments map with value to evaluate

    Returns
    -------
    Invoke[Dict[Any, Any]]
      An Invoke instance that will return the input dict with all its keys and
      values evaluated
    """
    return Invoke(
        lambda x: x,
        lambda context: {
            substitute(context, k): substitute(context, v)
            for k, v in mapping.items()
        },
    )
